#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""软件 PLC 主站：按工业扫描周期读写 plc_gateway，用于对拍与演示。

它是「PLC 程序」的替身，也是 IEC 61131-3 梯形图的对照实现：
    observe 模式  = 只读监视（相当于 PLC 只跑诊断块）
    control 模式  = 完整握手（相当于 PLC 跑自动流程）

握手时序（control 模式，与官方设备的「拍照 -> 识别 -> 按 Pos 运动」一致）
--------------------------------------------------------------------
    PLC: POS_SET := slot ; TASK_CMD := 1 (GOTO)      ; 下发去某位
    网关: -> /control/material_task(GOTO) -> 龙门运动 -> /sim/gantry_arrived
    网关: CAM_STATE := BUSY                          ; 到位待拍
    视觉: -> /vision/material_type -> 网关 CAM_STATE := DONE, CAM_DATA := type
    PLC: 读 CAM_DATA ; TASK_CMD := 2 (HANDLE)        ; 按类型放入
    网关: -> material_task(HANDLE) -> 抓放 -> /sim/handle_done
    网关: SHELF_BITS 置位, TASK_STATE := DONE

对应的梯形图写法（示意）
-----------------------
    |--[ DI_INPOS ]-------------------( CAM_REQ )--|
    |--[ CAM_STATE == 2 ]--[ TASK_CMD := 2 ]-------|
    |--[ TASK_STATE == 5 ]--[ next slot++ ]--------|

用法
----
    # 只监视（可与决策层 bt_runner 同时跑）
    python3 plc_master_demo.py --mode observe

    # 由本脚本代替决策层下发任务（必须先停 bt_runner，网关开 allow_control:=true）
    python3 plc_master_demo.py --mode control --loops 1
"""

import argparse
import socket
import struct
import sys
import time

# ---------------------------------------------------------------- 寄存器地址
CAM_STATE, CAM_DATA, CAM_MS = 1, 2, 3
TASK_CMD, POS_SET = 5, 10
X_ACT, Y_ACT, Z_ACT, AXIS_STATE = 11, 12, 13, 14
SHELF_BITS, SHELF_FULL, PLACED_CNT, TASK_STATE = 20, 21, 22, 23
HEARTBEAT, ERR = 30, 31

DI_INPOS, DI_CAM_DONE, DI_HELD, DI_FULL, DI_READY = 0, 1, 2, 3, 4

CAM_STATE_NAME = {0: "IDLE", 1: "BUSY", 2: "DONE", 3: "FAIL"}
AXIS_NAME = {0: "IDLE", 1: "MOVING", 2: "INPOS", 9: "FAULT"}
TASK_NAME = {0: "IDLE", 1: "GOTO", 2: "HANDLE", 5: "DONE", 9: "FAULT"}


class ModbusError(Exception):
    def __init__(self, fc, code):
        super().__init__("功能码 0x%02X 异常码 %d" % (fc, code))
        self.code = code


class ModbusClient:
    """最小 Modbus TCP 主站。与从站同源，只用标准库。"""

    def __init__(self, host, port, unit=1, timeout=1.5):
        self.sock = socket.create_connection((host, port), timeout)
        self.sock.settimeout(timeout)
        self.unit = unit
        self._tid = 0

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    def _recv(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("连接被对端关闭")
            buf += chunk
        return buf

    def _xfer(self, pdu):
        self._tid = (self._tid + 1) & 0xFFFF
        self.sock.sendall(struct.pack(">HHHB", self._tid, 0, len(pdu) + 1, self.unit) + pdu)
        tid, pid, length, unit = struct.unpack(">HHHB", self._recv(7))
        body = self._recv(length - 1)
        if body[0] & 0x80:
            raise ModbusError(body[0] & 0x7F, body[1] if len(body) > 1 else 0)
        return body

    # ---- 读 ----
    def read_holding(self, addr, count=1):
        body = self._xfer(struct.pack(">BHH", 3, addr, count))
        return list(struct.unpack(">" + "H" * count, body[2:2 + 2 * count]))

    def read_discrete(self, addr, count=1):
        body = self._xfer(struct.pack(">BHH", 2, addr, count))
        nbytes = body[1]
        raw = body[2:2 + nbytes]
        return [bool(raw[i // 8] & (1 << (i % 8))) for i in range(count)]

    def read_coils(self, addr, count=1):
        body = self._xfer(struct.pack(">BHH", 1, addr, count))
        nbytes = body[1]
        raw = body[2:2 + nbytes]
        return [bool(raw[i // 8] & (1 << (i % 8))) for i in range(count)]

    # ---- 写 ----
    def write_register(self, addr, value):
        self._xfer(struct.pack(">BHH", 6, addr, value & 0xFFFF))

    def write_registers(self, addr, values):
        pdu = struct.pack(">BHHB", 16, addr, len(values), 2 * len(values))
        pdu += struct.pack(">" + "H" * len(values), *[v & 0xFFFF for v in values])
        self._xfer(pdu)

    def write_coil(self, addr, value):
        self._xfer(struct.pack(">BHH", 5, addr, 0xFF00 if value else 0x0000))

    # ---- 辅助 ----
    def to_signed(self, u16):
        return u16 - 0x10000 if u16 >= 0x8000 else u16


def wait_for(mb, predicate, timeout_s, period_s=0.05, what=""):
    """轮询直到 predicate() 为真。返回等待耗时，超时抛 TimeoutError。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if predicate():
            return time.monotonic() - t0
        time.sleep(period_s)
    raise TimeoutError("等待 %s 超时（%.1f s）" % (what, timeout_s))


def run_observe(mb, interval, loops):
    """只读监视：把 PLC 能看到的一切打出来。"""
    print("观察模式（Ctrl-C 退出）。列含义：")
    print("  HB 心跳 | AX 轴状态 | CAM 相机状态 | POS 目标 | XYZ 实际(mm) | 货架 | 计数 | ERR\n")
    n = 0
    last_hb = None
    hb_t = time.monotonic()
    while loops <= 0 or n < loops:
        h = mb.read_holding(0, 32)
        bits = h[SHELF_BITS]
        filled = bin(bits).count("1")
        hb = h[HEARTBEAT]
        # 心跳 1 Hz，而轮询可能更快：只有「超过 2.5 s 没变」才算停滞，
        # 否则每两次轮询就误报一次。
        if last_hb is not None and hb != last_hb:
            hb_t = time.monotonic()
        stale = "（心跳停滞 >2.5s！）" if time.monotonic() - hb_t > 2.5 else ""
        last_hb = hb
        print("[%s] HB=%d%s AX=%-6s CAM=%-5s data=%d %4dms  X=%5d Y=%5d Z=%4d  "
              "货架=%s %2d/12  ERR=%d"
              % (time.strftime("%H:%M:%S"), hb, stale,
                 AXIS_NAME.get(h[AXIS_STATE], "?"),
                 CAM_STATE_NAME.get(h[CAM_STATE], "?"),
                 h[CAM_DATA], h[CAM_MS],
                 mb.to_signed(h[X_ACT]), mb.to_signed(h[Y_ACT]), mb.to_signed(h[Z_ACT]),
                 format(bits, "012b"), filled, h[ERR]))
        n += 1
        time.sleep(interval)


def run_control(mb, n_mat, timeout_inpos, timeout_cam, timeout_handle):
    """完整握手：代替决策层跑一轮。"""
    print("接管模式：本脚本代替决策层下发 GOTO/HANDLE。\n")
    t_start = time.monotonic()
    ok = 0
    for slot in range(1, n_mat + 1):
        t0 = time.monotonic()
        try:
            # 1) 去该库位：先写目标位，再写执行位（两个地址不相邻，必须分两次写）
            #    坑：POS_SET=10、TASK_CMD=5；寄存器 11 是只读的 X_ACT。
            #    写成 write_registers(10, [slot, 1]) 会把 1 写进 X_ACT，GOTO 根本没下发。
            mb.write_register(POS_SET, slot)
            mb.write_register(TASK_CMD, 1)               # 1 = GOTO
            wait_for(mb, lambda: mb.read_discrete(DI_INPOS, 1)[0],
                     timeout_inpos, what="轴到位")
            t_inpos = time.monotonic() - t0

            # 2) 等识别结果
            wait_for(mb, lambda: mb.read_holding(CAM_STATE, 1)[0] == 2,
                     timeout_cam, what="视觉结果")
            cam = mb.read_holding(CAM_DATA, 2)
            t_cam = time.monotonic() - t0

            # 3) 按类型抓放
            mb.write_register(TASK_CMD, 2)               # HANDLE
            wait_for(mb, lambda: mb.read_holding(TASK_STATE, 1)[0] == 5,
                     timeout_handle, what="抓放完成")
            t_done = time.monotonic() - t0
            ok += 1
            print("[第 %2d 轮] slot=%-2d 到位 %4.1fs  识别 type=%d (%dms, %4.1fs)  "
                  "抓放完成 %4.1fs"
                  % (slot, slot, t_inpos, cam[0], cam[1], t_cam, t_done))
        except (TimeoutError, ModbusError) as e:
            err = mb.read_holding(ERR, 1)[0]
            print("[第 %2d 轮] 失败：%s（网关 ERR=%d）" % (slot, e, err))

    total = time.monotonic() - t_start
    h = mb.read_holding(0, 32)
    filled = bin(h[SHELF_BITS]).count("1")
    print("\n汇总：成功 %d/%d，总耗时 %.1f s，货架 %s（%d/12），放满标志=%d"
          % (ok, n_mat, total, format(h[SHELF_BITS], "012b"), filled, h[SHELF_FULL]))
    return 0 if ok == n_mat else 1


def main():
    ap = argparse.ArgumentParser(description="软件 PLC 主站：读写 aic_plc_gateway")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5020)
    ap.add_argument("--unit", type=int, default=1)
    ap.add_argument("--mode", choices=["observe", "control"], default="observe")
    ap.add_argument("--interval", type=float, default=0.5, help="observe 模式轮询间隔 s")
    ap.add_argument("--loops", type=int, default=0, help="observe 模式次数，0=一直跑")
    ap.add_argument("--materials", type=int, default=12)
    ap.add_argument("--timeout-inpos", type=float, default=25.0)
    ap.add_argument("--timeout-cam", type=float, default=15.0)
    ap.add_argument("--timeout-handle", type=float, default=35.0)
    args = ap.parse_args()

    try:
        mb = ModbusClient(args.host, args.port, args.unit)
    except OSError as e:
        print("连不上 %s:%d —— %s" % (args.host, args.port, e), file=sys.stderr)
        print("先起网关：ros2 run aic_plc_gateway plc_gateway.py", file=sys.stderr)
        return 2

    try:
        hb = mb.read_holding(HEARTBEAT, 1)[0]
        print("已连接 %s:%d，网关心跳=%d\n" % (args.host, args.port, hb))
        if args.mode == "observe":
            run_observe(mb, args.interval, args.loops)
            return 0
        return run_control(mb, args.materials, args.timeout_inpos,
                           args.timeout_cam, args.timeout_handle)
    except KeyboardInterrupt:
        print("\n中断")
        return 130
    finally:
        mb.close()


if __name__ == "__main__":
    sys.exit(main())
