#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""PLC 网关：把 Gazebo 仿真的三轴/相机/货架状态映射成 PLC 变量。

参考的官方架构（见 docs/vnext_pdf 与两份菲尼克斯手册）
----------------------------------------------------
  海康相机 --TCP/IP--> 算法容器(Python)
                          |  PLCnext RSC (PyPlcnextRsc / gRPC)
                          v
                       vPLC (IEC 61131-3, RK3568 上的 Podman 容器)
                          |  PROFINET
                          v
                       AXL 耦合器 + Inline DI/DO --> ESTUN 伺服 x3

官方数据契约是一个结构体变量 ``Camera``：
  Camera.Request   PLC -> 算法   请求拍照
  Camera.Data      算法 -> PLC   物料编号，PLC 再映射成 ``Pos`` 决定货架位置
本网关的寄存器/变量命名与它一一对应，不做自创命名，便于将来换上真 PLC。

为什么 Modbus 而不是直接 RSC
---------------------------
RSC 是 PLCnext 专有（gRPC/HTTPS），没有 vPLC 硬件时无法真实联调；
Modbus TCP 是任何 PLC 都有的通用口，且本实现只用标准库（socket/struct/threading），
现场机器没网也能跑。两者的**变量语义相同**，区别只在传输层，见 §「与 RSC 的对应」。

协议实现范围（工业主站最常用的子集）
------------------------------------
  FC 01/02/03/04 读线圈 / 离散输入 / 保持寄存器 / 输入寄存器
  FC 05/06       写单线圈 / 单寄存器
  FC 15/16       写多线圈 / 多寄存器

MBAP 报头（7 字节，大端）
------------------------
  transaction_id 2B | protocol_id 2B(=0) | length 2B | unit_id 1B | PDU...

寄存器映射（偏移从 0 起；PLC 软件里通常显示为 40001+偏移）
--------------------------------------------------------
  [保持寄存器 4x —— PLC 读写]
   -- Camera 结构体（对齐官方）--
    0  CAM_REQ      PLC→算法  1=请求拍照（网关取走后自动清 0）
    1  CAM_STATE    算法→PLC  0=IDLE 1=BUSY 2=DONE 3=FAIL
    2  CAM_DATA     算法→PLC  物料编号 1..4，0=未识别（官方 Camera.Data）
    3  CAM_MS       算法→PLC  本次识别耗时 ms
    4  CAM_ERR      算法→PLC  错误码（0=无）
    5  TASK_CMD     PLC→控制  1=GOTO 2=HANDLE（配合 POS_SET，执行后自动清 0）
   -- 目标位与三轴（对齐官方 Pos + 伺服反馈）--
   10  POS_SET      PLC→控制  目标库位 1..12（官方变量 Pos）
   11  X_ACT        控制→PLC  X 实际位置 mm（**int16 补码**）
   12  Y_ACT        控制→PLC  Y 实际位置 mm（int16 补码）
   13  Z_ACT        控制→PLC  Z 实际位置 mm（int16 补码）
   14  AXIS_STATE   0=IDLE 1=MOVING 2=INPOS 9=FAULT
   -- 货架与计数（对齐 eHMI 的「库位点亮 / 放满提示」）--
   20  SHELF_BITS   货架占用位图，bit(n-1)=第 n 号物料已入位
   21  SHELF_FULL   1=放满
   22  PLACED_CNT   已放入计数
   23  TASK_STATE   0=IDLE 1=GOTO 2=HANDLE 5=DONE 9=FAULT
   -- 诊断 --
   30  HEARTBEAT    每秒 +1（回绕），PLC 用它判断网关是否活着
   31  ERR          最近错误码

  [线圈 0x —— PLC 写，网关执行后自动清]
    0  COIL_TRIGGER  上升沿 -> 触发一次拍照
    1  COIL_ATTACH   上升沿 -> 吸盘吸附
    2  COIL_DETACH   上升沿 -> 吸盘释放
    3  COIL_RESET    上升沿 -> 清计数与状态

  [离散输入 1x —— 只读]
    0  DI_INPOS    轴已到位（可拍照）
    1  DI_CAM_DONE 拍照结果就绪
    2  DI_HELD     吸盘持有物料（逻辑标志，非真空传感器）
    3  DI_FULL     货架放满
    4  DI_READY    控制就绪

两种模式
--------
  allow_control:=false（默认）  观察模式：PLC 只读；写 CMD/线圈被忽略并提示。
                                可与决策层 bt_runner 同时跑，互不干扰。
  allow_control:=true           接管模式：PLC 写的 POS/CAM_REQ 转成 ROS 指令。
                                此时必须停掉 bt_runner，否则两条指令流会打架。

与 RSC 的对应
-------------
  RSC 的 read_variable("Camera.Data")  ≈ 读保持寄存器 2
  RSC 的 write_variable("Camera.Request", 1) ≈ 写保持寄存器 0
  变量树 Camera.* / Pos / Axis.* 与上表分组一一对应；换真机时只换传输层，
  PLC 侧程序（IEC 61131-3）与算法侧逻辑都不用改。
"""

import socket
import struct
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String, UInt8

from aic_msgs.msg import MaterialTask

# ---------------------------------------------------------------- 保持寄存器
CAM_REQ = 0
CAM_STATE = 1
CAM_DATA = 2
CAM_MS = 3
CAM_ERR = 4
TASK_CMD = 5

POS_SET = 10
X_ACT = 11
Y_ACT = 12
Z_ACT = 13
AXIS_STATE = 14

SHELF_BITS = 20
SHELF_FULL = 21
PLACED_CNT = 22
TASK_STATE = 23

HEARTBEAT = 30
ERR = 31
HR_COUNT = 40

# ---------------------------------------------------------------- 线圈 / 输入
COIL_TRIGGER = 0
COIL_ATTACH = 1
COIL_DETACH = 2
COIL_RESET = 3
COIL_COUNT = 16

DI_INPOS = 0
DI_CAM_DONE = 1
DI_HELD = 2
DI_FULL = 3
DI_READY = 4
DI_COUNT = 16

IR_COUNT = 16

# 状态码
CAM_IDLE, CAM_BUSY, CAM_DONE, CAM_FAIL = 0, 1, 2, 3
AX_IDLE, AX_MOVING, AX_INPOS, AX_FAULT = 0, 1, 2, 9
TK_IDLE, TK_GOTO, TK_HANDLE, TK_DONE, TK_FAULT = 0, 1, 2, 5, 9

# 错误码
ERR_NONE = 0
ERR_NO_SLOT = 1
ERR_NO_STAGED = 2
ERR_OBSERVE = 3
ERR_NO_TYPE = 4        # HANDLE 时没有有效识别结果


def _now():
    return time.monotonic()


def to_int16_mm(v_m):
    """米 -> 毫米并压到 int16 范围。寄存器按补码存放，PLC 侧按 INT 解读。"""
    v = int(round(float(v_m) * 1000.0))
    return max(-32768, min(32767, v))


# ---------------------------------------------------------------- 数据区
class DataStore:
    """四张 Modbus 表的线程安全容器。

    ROS 定时器线程回填状态，socket 线程响应主站读写，两边都要加锁。
    用 RLock 是为了让「读多个表」的复合操作能嵌套加锁。
    """

    def __init__(self):
        self.lock = threading.RLock()
        self.coils = [False] * COIL_COUNT
        self.discrete = [False] * DI_COUNT
        self.holding = [0] * HR_COUNT
        self.input = [0] * IR_COUNT

    def read_bits(self, table, addr, count):
        with self.lock:
            return list(table[addr:addr + count])

    def read_coils(self, addr, count):
        return self.read_bits(self.coils, addr, count)

    def read_discrete(self, addr, count):
        return self.read_bits(self.discrete, addr, count)

    def write_coil(self, addr, value):
        with self.lock:
            self.coils[addr] = bool(value)

    def write_coils(self, addr, values):
        with self.lock:
            for i, v in enumerate(values):
                self.coils[addr + i] = bool(v)

    def read_holding(self, addr, count):
        with self.lock:
            return list(self.holding[addr:addr + count])

    def read_input(self, addr, count):
        with self.lock:
            return list(self.input[addr:addr + count])

    def write_holding(self, addr, value):
        with self.lock:
            self.holding[addr] = value & 0xFFFF

    def write_holdings(self, addr, values):
        with self.lock:
            for i, v in enumerate(values):
                self.holding[addr + i] = v & 0xFFFF


# ---------------------------------------------------------------- Modbus 从站
class ModbusTcpSlave:
    """极简 Modbus TCP 从站：一个 accept 线程 + 每连接一个处理线程。"""

    def __init__(self, store, host, port, unit_id, log):
        self.store = store
        self.unit_id = unit_id
        self.log = log
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, port))
        self._srv.listen(4)
        self._srv.settimeout(0.5)      # 让 accept 定期回看 _running
        self._running = False
        self.requests = 0
        self.errors = 0

    def start(self):
        self._running = True
        threading.Thread(target=self._accept_loop, daemon=True,
                         name="modbus-accept").start()

    def stop(self):
        self._running = False
        try:
            self._srv.close()
        except OSError:
            pass

    def _accept_loop(self):
        while self._running:
            try:
                conn, peer = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            # 空闲超时必须远大于主站扫描周期：曾用 2.0 s，而主站轮询间隔也是 2.0 s，
            # 两边赛跑 -> 网关先超时断开，主站第二条请求就撞上「连接被对端关闭」。
            # 工业从站不会因为空闲主动断链，这里给 30 s（stop 靠 daemon 线程退出）。
            conn.settimeout(30.0)
            threading.Thread(target=self._serve, args=(conn, peer), daemon=True,
                             name="modbus-client").start()

    @staticmethod
    def _recv_exact(conn, n):
        buf = b""
        while len(buf) < n:
            try:
                chunk = conn.recv(n - len(buf))
            except socket.timeout:
                return None
            if not chunk:
                return None
            buf += chunk
        return buf

    def _serve(self, conn, peer):
        self.log("PLC 主站接入 %s:%d" % peer)
        try:
            while self._running:
                hdr = self._recv_exact(conn, 7)
                if hdr is None:
                    break
                tid, pid, length, unit = struct.unpack(">HHHB", hdr)
                if pid != 0 or length < 2 or length > 260:
                    self.errors += 1
                    break
                pdu = self._recv_exact(conn, length - 1)
                if pdu is None:
                    break
                resp = self._dispatch(pdu, unit)
                if resp is None:
                    continue
                conn.sendall(struct.pack(">HHHB", tid, 0, len(resp) + 1, unit) + resp)
                self.requests += 1
        except (ConnectionResetError, BrokenPipeError, OSError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass
            self.log("PLC 主站断开 %s:%d" % peer)

    # ---- 功能码分发 ----
    def _dispatch(self, pdu, unit):
        if unit not in (0, self.unit_id):
            return self._exc(pdu[0], 11)
        fc = pdu[0]
        try:
            if fc == 0x01:
                return self._fc_read_bits(pdu, self.store.read_coils, 1, COIL_COUNT)
            if fc == 0x02:
                return self._fc_read_bits(pdu, self.store.read_discrete, 2, DI_COUNT)
            if fc == 0x03:
                return self._fc_read_regs(pdu, self.store.read_holding, 3, HR_COUNT)
            if fc == 0x04:
                return self._fc_read_regs(pdu, self.store.read_input, 4, IR_COUNT)
            if fc == 0x05:
                return self._fc_write_coil(pdu)
            if fc == 0x06:
                return self._fc_write_reg(pdu)
            if fc == 0x0F:
                return self._fc_write_coils(pdu)
            if fc == 0x10:
                return self._fc_write_regs(pdu)
        except (struct.error, IndexError):
            self.errors += 1
            return self._exc(fc, 3)
        return self._exc(fc, 1)

    @staticmethod
    def _exc(fc, code):
        return bytes([(fc | 0x80) & 0xFF, code])

    def _fc_read_bits(self, pdu, reader, fc, limit):
        addr, count = struct.unpack(">HH", pdu[1:5])
        if not 1 <= count <= 2000:
            return self._exc(fc, 3)
        if addr + count > limit:
            return self._exc(fc, 2)
        bits = reader(addr, count)
        nbytes = (count + 7) // 8
        out = bytearray(nbytes)
        for i, b in enumerate(bits):
            if b:
                out[i // 8] |= 1 << (i % 8)   # Modbus 位序：低地址位在低位
        return bytes([fc, nbytes]) + bytes(out)

    def _fc_read_regs(self, pdu, reader, fc, limit):
        addr, count = struct.unpack(">HH", pdu[1:5])
        if not 1 <= count <= 125:
            return self._exc(fc, 3)
        if addr + count > limit:
            return self._exc(fc, 2)
        vals = reader(addr, count)
        return bytes([fc, 2 * count]) + struct.pack(">" + "H" * count, *vals)

    def _fc_write_coil(self, pdu):
        addr, val = struct.unpack(">HH", pdu[1:5])
        if addr >= COIL_COUNT:
            return self._exc(5, 2)
        if val not in (0x0000, 0xFF00):
            return self._exc(5, 3)
        self.store.write_coil(addr, val == 0xFF00)
        return bytes(pdu[:5])                 # 原样回显

    def _fc_write_reg(self, pdu):
        addr, val = struct.unpack(">HH", pdu[1:5])
        if addr >= HR_COUNT:
            return self._exc(6, 2)
        self.store.write_holding(addr, val)
        return bytes(pdu[:5])

    def _fc_write_coils(self, pdu):
        addr, count, nbytes = struct.unpack(">HHB", pdu[1:6])
        if not 1 <= count <= 1968 or nbytes != (count + 7) // 8:
            return self._exc(15, 3)
        if addr + count > COIL_COUNT:
            return self._exc(15, 2)
        raw = pdu[6:6 + nbytes]
        vals = [bool(raw[i // 8] & (1 << (i % 8))) for i in range(count)]
        self.store.write_coils(addr, vals)
        return struct.pack(">BHH", 15, addr, count)

    def _fc_write_regs(self, pdu):
        addr, count, nbytes = struct.unpack(">HHB", pdu[1:6])
        if not 1 <= count <= 123 or nbytes != 2 * count:
            return self._exc(16, 3)
        if addr + count > HR_COUNT:
            return self._exc(16, 2)
        vals = struct.unpack(">" + "H" * count, pdu[6:6 + 2 * count])
        self.store.write_holdings(addr, vals)
        return struct.pack(">BHH", 16, addr, count)


# ---------------------------------------------------------------- ROS 节点
# ---------------------------------------------------------------- 后端抽象
# 语义标签 -> (Modbus 表, 偏移)。状态机只认标签名，不认识寄存器也不认识 PLC 变量路径：
# Modbus 后端按表+偏移读写，RSC 后端按 config/plc_rsc_map.yaml 的路径读写。
TAG_ADDR = {
    "CAM_REQ": ("holding", CAM_REQ),
    "CAM_STATE": ("holding", CAM_STATE),
    "CAM_DATA": ("holding", CAM_DATA),
    "CAM_MS": ("holding", CAM_MS),
    "CAM_ERR": ("holding", CAM_ERR),
    "TASK_CMD": ("holding", TASK_CMD),
    "POS_SET": ("holding", POS_SET),
    "X_ACT": ("holding", X_ACT),
    "Y_ACT": ("holding", Y_ACT),
    "Z_ACT": ("holding", Z_ACT),
    "AXIS_STATE": ("holding", AXIS_STATE),
    "SHELF_BITS": ("holding", SHELF_BITS),
    "SHELF_FULL": ("holding", SHELF_FULL),
    "PLACED_CNT": ("holding", PLACED_CNT),
    "TASK_STATE": ("holding", TASK_STATE),
    "HEARTBEAT": ("holding", HEARTBEAT),
    "ERR": ("holding", ERR),
    "COIL_TRIGGER": ("coil", COIL_TRIGGER),
    "COIL_ATTACH": ("coil", COIL_ATTACH),
    "COIL_DETACH": ("coil", COIL_DETACH),
    "COIL_RESET": ("coil", COIL_RESET),
    "DI_INPOS": ("discrete", DI_INPOS),
    "DI_CAM_DONE": ("discrete", DI_CAM_DONE),
    "DI_HELD": ("discrete", DI_HELD),
    "DI_FULL": ("discrete", DI_FULL),
    "DI_READY": ("discrete", DI_READY),
}

# 每个 tick 要读的输入（PLC 写、网关读）。顺序无关，只是让读取集中在一处。
INPUT_TAGS = ("CAM_REQ", "TASK_CMD", "POS_SET",
              "COIL_TRIGGER", "COIL_ATTACH", "COIL_DETACH", "COIL_RESET")


class IoBackend:
    """PLC I/O 后端。状态机只调 read()/write()，换协议不动状态机。"""

    name = "base"

    def open(self):
        raise NotImplementedError

    def close(self):
        pass

    def read(self):
        """读一次全部输入，返回 {标签: 值}。缺的标签由调用方按默认值处理。"""
        raise NotImplementedError

    def write(self, values):
        """把状态写到 PLC。values 的键必须在 TAG_ADDR 里。"""
        raise NotImplementedError

    def can_write(self, tag):
        """该标签能不能由网关写。Modbus 四张表都能写；RSC 要看标签表里的 dir。"""
        return True

    def describe(self):
        return self.name


class ModbusBackend(IoBackend):
    """Modbus TCP 从站：标签落在四张数据表里，主站通过 socket 读写。"""

    name = "modbus"

    def __init__(self, host, port, unit_id, log):
        self.host, self.port, self.unit_id = host, int(port), int(unit_id)
        self.store = DataStore()
        self.slave = ModbusTcpSlave(self.store, host, self.port, self.unit_id, log)

    def open(self):
        self.slave.start()

    def close(self):
        self.slave.stop()

    def _table(self, name):
        return {"holding": self.store.holding,
                "coil": self.store.coils,
                "discrete": self.store.discrete}[name]

    def read(self):
        with self.store.lock:
            out = {}
            for tag in INPUT_TAGS:
                table, off = TAG_ADDR[tag]
                out[tag] = self._table(table)[off]
            return out

    def write(self, values):
        # 有符号量（坐标 mm）在这里才折成 uint16：RSC 后端要的是真负数，不能提前掩码。
        with self.store.lock:
            for tag, v in values.items():
                table, off = TAG_ADDR[tag]
                if table == "holding":
                    self._table(table)[off] = int(v) & 0xFFFF
                else:
                    self._table(table)[off] = bool(v)

    def describe(self):
        return "modbus tcp %s:%d unit=%d" % (self.host, self.port, self.unit_id)


class RscBackend(IoBackend):
    """PLCnext RSC：一次 Read 取全部输入，写只发变化过的标签。

    真机前置条件：PLC 侧变量名与 config/plc_rsc_map.yaml 逐字一致
    （见 docs/guides/plcnext_rsc_setup.md §2）。没有 vPLC 时用 rsc_dry_run=true
    走内存假 PLC，可完整验证映射与状态机，但证明不了现场变量名对不对。
    """

    name = "rsc"

    def __init__(self, map_path, host, port, user, password, dry_run, timeout_s=5.0):
        from rsc_link import RscLink, RscMap          # 只有选 RSC 才需要这个模块
        self.map_path = map_path
        self.link = RscLink(RscMap.load(map_path), host, int(port),
                            user or None, password or None,
                            timeout_s=float(timeout_s), dry_run=bool(dry_run))

    def open(self):
        self.link.connect()

    def close(self):
        self.link.close()

    def read(self):
        return self.link.read_all()

    def write(self, values):
        # write_all 只写发生变化的标签（心跳每秒变，其余按需），省往返
        self.link.write_all(values)

    def can_write(self, tag):
        # dir=plc2gw 的变量归 PLC 所有，rsc_link.write_all 会直接跳过（见 rsc_link.py:428）。
        # 也就是说 write({"CAM_REQ": 0}) 会「成功返回但什么都没写」——必须显式暴露出来，
        # 否则上层会把「清请求位」当成已完成，实际上 PLC 侧的电平一直悬着。
        spec = self.link.map.tags.get(tag)
        return spec is not None and spec.direction in ("gw2plc", "both")

    def describe(self):
        mode = "干跑（内存假 PLC）" if self.link.dry_run else self.link.host
        return "rsc %s  标签表=%s" % (mode, self.map_path)


class PlcGateway(Node):
    def __init__(self):
        super().__init__("plc_gateway")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5020)        # 502 需 root，默认 5020
        self.declare_parameter("unit_id", 1)
        self.declare_parameter("allow_control", False)
        self.declare_parameter("poll_hz", 50.0)
        self.declare_parameter("n_materials", 12)
        self.declare_parameter("task_topic", "/control/material_task")
        self.declare_parameter("joint_states_topic",
                              "/world/aic_warehouse/model/aic_gantry/joint_state")
        self.declare_parameter("arrived_topic", "/sim/gantry_arrived")
        self.declare_parameter("handled_topic", "/sim/handle_done")
        self.declare_parameter("ready_topic", "/sim/gantry_ready")
        self.declare_parameter("vision_result_topic", "/vision/material_type")
        self.declare_parameter("vision_trigger_topic", "/vision/trigger")

        # ---- 协议后端（modbus | rsc）----
        self.declare_parameter("backend", "modbus")
        self.declare_parameter("rsc_host", "192.168.3.99")
        self.declare_parameter("rsc_port", 41100)
        self.declare_parameter("rsc_user", "admin")
        self.declare_parameter("rsc_password", "plcnext")
        self.declare_parameter("rsc_map", "")        # 空 = 用包里默认标签表
        self.declare_parameter("rsc_dry_run", False) # true = 内存假 PLC

        p = lambda n: self.get_parameter(n).value
        self.allow_control = bool(p("allow_control"))
        self.n_mat = int(p("n_materials"))

        # ---- 内部状态 ----
        self.cam_state = CAM_IDLE
        self.cam_t0 = None
        self.cam_slot = 0
        self.axis_state = AX_IDLE
        self.task_state = TK_IDLE
        self.staged = 0          # 置物台上的物料号，0=空
        self.cam_type = 0        # 最近一次识别结果 1..4，0=未识别
        self.cam_ms = 0          # 本次识别耗时，随状态一起写给 PLC
        self.cam_err = 0
        self.placed_mask = 0     # bit(n-1) = 第 n 号已入位
        self.placed = 0
        self.goto_cnt = 0
        self.visit = 0
        self.err = ERR_NONE
        self.held = False
        self.ready = False
        self.actual = {"x": 0.0, "y": 0.0, "z": 0.0}

        self._last_cmd = 0
        self._last_cam_req = False
        self._last_trigger = False
        self._last_attach = False
        self._last_detach = False
        self._last_reset = False
        self._self_pub_until = 0.0
        self._warned_observe = False
        self._warned_unwritable = set()   # 已经提示过「这个位网关写不动」的标签
        self._hb = 0
        self._hb_t = _now()

        # ---- ROS 接口 ----
        task_topic = p("task_topic")
        self.task_pub = self.create_publisher(MaterialTask, task_topic, 10)
        self.create_subscription(MaterialTask, task_topic, self._on_task, 10)
        self.trigger_pub = self.create_publisher(Empty, p("vision_trigger_topic"), 10)
        self.create_subscription(String, p("vision_result_topic"), self._on_vision, 10)

        self.attach_pub = {i: self.create_publisher(Empty, "/sim/attach_{:02d}".format(i), 10)
                           for i in range(1, self.n_mat + 1)}
        self.detach_pub = {i: self.create_publisher(Empty, "/sim/detach_{:02d}".format(i), 10)
                           for i in range(1, self.n_mat + 1)}

        # 坑：Gazebo 的 joint_state 发布器是 BEST_EFFORT，订阅端用默认 RELIABLE
        # 会**静默不匹配**——不报错，但一帧也收不到，表现为坐标永远 0。
        js_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, p("joint_states_topic"),
                                 self._on_joint_state, js_qos)
        self.create_subscription(UInt8, p("arrived_topic"), self._on_arrived, 10)
        self.create_subscription(UInt8, p("handled_topic"), self._on_handled, 10)
        self.create_subscription(Bool, p("ready_topic"), self._on_ready, 10)

        # ---- 协议后端（modbus | rsc）----
        self.io = self._make_backend(p)
        try:
            self.io.open()
        except OSError as e:
            self.get_logger().error(
                "Modbus 端口 %d 绑定失败：%s（被占用？低于 1024 需 root）"
                % (int(p("port")), e))
            raise
        except Exception as e:      # RSC 连不上 / 标签表读不了
            self.get_logger().error("协议后端 %s 启动失败：%s" % (p("backend"), e))
            raise

        self.create_timer(1.0 / max(5.0, float(p("poll_hz"))), self._tick)

        self.get_logger().info(
            "PLC 网关就绪：%s，模式=%s"
            % (self.io.describe(),
               "接管" if self.allow_control else "观察（只读，可与 bt_runner 共存）"))

    # ---- 内部 ----
    def _make_backend(self, p):
        """按 backend 参数造后端。换协议只改这里，状态机不认协议。"""
        kind = str(p("backend")).strip().lower()
        if kind == "modbus":
            return ModbusBackend(str(p("host")), int(p("port")), int(p("unit_id")),
                                 self._slave_log)
        if kind == "rsc":
            map_path = str(p("rsc_map")).strip()
            if not map_path:
                import rsc_link
                map_path = str(rsc_link._default_map())
            return RscBackend(map_path, str(p("rsc_host")), int(p("rsc_port")),
                              str(p("rsc_user")), str(p("rsc_password")),
                              bool(p("rsc_dry_run")))
        raise ValueError("backend 只能是 modbus 或 rsc，收到 %r" % kind)

    def _slave_log(self, text):
        self.get_logger().info("[modbus] " + text)

    def _set_err(self, code, text):
        self.err = code
        self.get_logger().warn("ERR=%d %s" % (code, text))

    # ---- 订阅 ----
    def _on_joint_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            axis = name.split("_")[0]
            if axis in self.actual:
                self.actual[axis] = float(pos)

    def _on_task(self, msg):
        if _now() > self._self_pub_until and self.allow_control:
            self.get_logger().warn(
                "接管模式下收到外部 %s(slot=%d)：bt_runner 仍在跑？"
                % (msg.action, msg.slot_id), throttle_duration_sec=5.0)
        if msg.action == MaterialTask.ACTION_GOTO:
            self.goto_cnt += 1
            self.staged = int(msg.slot_id)
            self.task_state = TK_GOTO
            self.axis_state = AX_MOVING
        elif msg.action == MaterialTask.ACTION_HANDLE:
            self.task_state = TK_HANDLE
            self.axis_state = AX_MOVING
        elif msg.action == MaterialTask.ACTION_HOME:
            self.task_state = TK_GOTO
            self.axis_state = AX_MOVING

    def _on_arrived(self, msg):
        """轴到位 = 官方流程里的「可以拍照」。视觉节点会被控制器自动触发。"""
        self.axis_state = AX_INPOS
        self.task_state = TK_GOTO
        self.cam_slot = int(msg.data)
        self.cam_state = CAM_BUSY
        self.cam_t0 = _now()

    def _on_vision(self, msg):
        """视觉结果：String，内容为 "1".."4"。"""
        try:
            tid = int(str(msg.data).strip())
        except ValueError:
            self.cam_state = CAM_FAIL
            self._set_err(ERR_NONE, "视觉结果无法解析：%r" % msg.data)
            return
        self.cam_state = CAM_DONE
        self.cam_type = tid if tid in (1, 2, 3, 4) else 0
        # 识别结果先落到本地状态，由 _publish_status 统一写给 PLC：
        # 回调线程不直接碰协议层——RSC 是 TCP 往返，两个线程同时写会串包。
        self.cam_ms = int((_now() - self.cam_t0) * 1000) if self.cam_t0 else 0
        self.cam_err = 0

    def _on_handled(self, msg):
        slot = int(msg.data)
        self.placed += 1
        if 1 <= slot <= 16:
            self.placed_mask |= (1 << (slot - 1))
        self.staged = 0
        self.held = False
        self.cam_type = 0
        self.task_state = TK_DONE
        self.axis_state = AX_MOVING

    def _on_ready(self, msg):
        self.ready = bool(msg.data)

    # ---- 定时器 ----
    def _tick(self):
        self._poll_commands()
        self._publish_status()

    def _poll_commands(self):
        try:
            st = self.io.read()
        except Exception as e:      # RSC 断链 / 网络抖动：本轮跳过，别把节点搞崩
            self.get_logger().warn("读取 PLC 输入失败：%s" % e, throttle_duration_sec=5.0)
            return

        task_cmd = int(st.get("TASK_CMD", 0) or 0)
        pos = int(st.get("POS_SET", 0) or 0)
        cam_req = bool(st.get("CAM_REQ", False))
        trig = bool(st.get("COIL_TRIGGER", False))
        att = bool(st.get("COIL_ATTACH", False))
        det = bool(st.get("COIL_DETACH", False))
        rst = bool(st.get("COIL_RESET", False))

        # 任务：先写 POS_SET 设定目标，再写 TASK_CMD 触发（PLC 的「设定值 + 执行位」）
        # Modbus 下网关把 TASK_CMD 清 0 作为应答；RSC 下该标签是 PLC 写的，写不动，
        # 需要 PLC 程序自己把 TASK_CMD 复位（否则只有边沿变化才会再次触发）。
        # 坑：只有真执行了才应答。观察模式下 _exec_task 什么都不做，
        # 若照样清 0 就是替真控制器「应答」掉一条没人执行的命令。
        if task_cmd and task_cmd != self._last_cmd:
            if self._exec_task(task_cmd, pos):
                self._ack("TASK_CMD")
        self._last_cmd = task_cmd

        # 拍照请求：只认上升沿。CAM_REQ 是 PLC 的电平信号，RSC 下它归 PLC 所有
        # （dir=plc2gw），网关清不掉。若不判边沿，只要 PLC 让它保持 1，
        # 每个 tick 都会重发触发、并把 cam_state 反复打回 BUSY ——
        # PLC 永远等不到 DONE，现象是「刚识别完状态又变忙」。
        if cam_req:
            if not self.allow_control:
                self.get_logger().warn(
                    "PLC 置了 CAM_REQ，但网关在观察模式（allow_control=false），"
                    "不会触发拍照；要由 PLC 主导请用 allow_control:=true",
                    throttle_duration_sec=5.0)
            elif not self._last_cam_req:
                self.trigger_pub.publish(Empty())
                self.cam_state = CAM_BUSY
                self.cam_t0 = _now()
                self._ack("CAM_REQ")
                self.get_logger().info("PLC 请求拍照 -> 已触发 vision")
        self._last_cam_req = cam_req

        if trig and not self._last_trigger:
            if self.allow_control:
                self.trigger_pub.publish(Empty())
                self._ack("COIL_TRIGGER")
            else:
                self.get_logger().warn("观察模式忽略 COIL_TRIGGER",
                                       throttle_duration_sec=5.0)
        self._last_trigger = trig

        if att and not self._last_attach:
            if self._exec_attach():
                self._ack("COIL_ATTACH")
        self._last_attach = att

        if det and not self._last_detach:
            if self._exec_detach():
                self._ack("COIL_DETACH")
        self._last_detach = det

        if rst and not self._last_reset:
            self.placed = 0
            self.placed_mask = 0
            self.staged = 0
            self.held = False
            self.err = ERR_NONE
            self.task_state = TK_IDLE
            self.cam_state = CAM_IDLE
            self.cam_type = 0
            if self.allow_control:
                self._ack("COIL_RESET")
            else:
                self.get_logger().warn("观察模式忽略 COIL_RESET",
                                       throttle_duration_sec=5.0)
            self.get_logger().info("PLC 复位")
        self._last_reset = rst

    def _ack(self, tag):
        """把 PLC 的请求位清 0。写不动就一次性说明白，别让它静默悬着。

        坑：RSC 标签表里 CAM_REQ / COIL_* / TASK_CMD 全是 dir=plc2gw（归 PLC 所有），
        rsc_link.write_all 会直接跳过这些标签——write() 正常返回，PLC 侧却没有任何变化。
        这种后端下必须由 PLC 程序自己复位，否则该请求只有一次上升沿，
        第二次请求永远等不到。
        """
        if not self.io.can_write(tag):
            if tag not in self._warned_unwritable:
                self._warned_unwritable.add(tag)
                self.get_logger().warn(
                    "%s 在当前后端归 PLC 所有（dir=plc2gw），网关清 0 无效；"
                    "PLC 程序必须自己复位该位，否则同一请求只能触发一次" % tag)
            return False
        table, _ = TAG_ADDR[tag]
        self.io.write({tag: 0 if table == "holding" else False})
        return True

    def _exec_task(self, cmd, slot):
        """PLC 动作：1=GOTO 2=HANDLE。返回 True = 真的下发了（可以应答 TASK_CMD）。"""
        if cmd not in (1, 2):
            self._set_err(ERR_NONE, "未知 TASK_CMD=%d，忽略" % cmd)
            return False
        if not self.allow_control:
            if not self._warned_observe:
                self.get_logger().warn(
                    "观察模式收到 TASK_CMD=%d，已忽略。要由 PLC 下发任务请用 "
                    "allow_control:=true（并停掉 bt_runner）" % cmd)
                self._warned_observe = True
            self._set_err(ERR_OBSERVE, "观察模式不接受控制写入")
            return False
        if not 1 <= slot <= self.n_mat:
            self._set_err(ERR_NO_SLOT, "Pos=%d 越界（应 1..%d）" % (slot, self.n_mat))
            return False
        # HANDLE 必须带类型：控制器按 material_type 决定放进哪一列。
        # 不带（或 0）会走「未识别」快速分支——不动、不入架，但照样回 handled，
        # 于是 PLC 侧看到 TASK_STATE=DONE 却什么都没放进去（这个坑很隐蔽）。
        if cmd == 2 and self.cam_type not in (1, 2, 3, 4):
            self._set_err(ERR_NO_TYPE, "HANDLE 时无有效识别结果（CAM_DATA=%d）"
                          % self.cam_type)
            return False
        if cmd == 1:                 # 新一轮上料：作废上一轮的识别结果
            self.cam_type = 0
            self.cam_state = CAM_IDLE
        self.visit = (self.visit % 255) + 1
        msg = MaterialTask()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.action = MaterialTask.ACTION_GOTO if cmd == 1 else MaterialTask.ACTION_HANDLE
        msg.slot_id = slot
        msg.visit_index = self.visit
        msg.material_type = self.cam_type if cmd == 2 else MaterialTask.MATERIAL_UNKNOWN
        msg.note = "from PLC"
        self._self_pub_until = _now() + 0.2
        self.task_pub.publish(msg)
        self.get_logger().info("PLC -> %s slot=%d type=%d"
                               % (msg.action, slot, msg.material_type))
        return True

    def _exec_attach(self):
        """返回 True = 真的执行了（可以应答 COIL_ATTACH）。"""
        if not self.allow_control:
            self.get_logger().warn("观察模式忽略 COIL_ATTACH", throttle_duration_sec=5.0)
            return False
        if not 1 <= self.staged <= self.n_mat:
            self._set_err(ERR_NO_STAGED, "台面无料，无法吸附")
            return False
        self.attach_pub[self.staged].publish(Empty())
        self.held = True
        self.axis_state = AX_MOVING
        self.get_logger().info("PLC -> attach material_%02d" % self.staged)
        return True

    def _exec_detach(self):
        """返回 True = 真的执行了（可以应答 COIL_DETACH）。"""
        if not self.allow_control:
            self.get_logger().warn("观察模式忽略 COIL_DETACH", throttle_duration_sec=5.0)
            return False
        if not 1 <= self.staged <= self.n_mat:
            self._set_err(ERR_NO_STAGED, "台面无料，无法释放")
            return False
        self.detach_pub[self.staged].publish(Empty())
        self.held = False
        self.axis_state = AX_MOVING
        self.get_logger().info("PLC -> detach material_%02d" % self.staged)
        return True

    def _publish_status(self):
        now = _now()
        if now - self._hb_t >= 1.0:
            self._hb = (self._hb + 1) & 0xFFFF
            self._hb_t = now
        full = (self.placed_mask & ((1 << self.n_mat) - 1)) == ((1 << self.n_mat) - 1)
        try:
            self.io.write({
                "HEARTBEAT": self._hb,
                "CAM_STATE": self.cam_state,
                "CAM_DATA": self.cam_type,
                "CAM_MS": self.cam_ms,
                "CAM_ERR": self.cam_err,
                # 坐标给真负数：折成 uint16 是 Modbus 后端的事，RSC 要的是 INT
                "X_ACT": to_int16_mm(self.actual["x"]),
                "Y_ACT": to_int16_mm(self.actual["y"]),
                "Z_ACT": to_int16_mm(self.actual["z"]),
                "AXIS_STATE": self.axis_state,
                "SHELF_BITS": self.placed_mask,
                "SHELF_FULL": 1 if full else 0,
                "PLACED_CNT": self.placed,
                "TASK_STATE": self.task_state,
                "ERR": self.err,
                "DI_INPOS": self.axis_state == AX_INPOS,
                "DI_CAM_DONE": self.cam_state == CAM_DONE,
                "DI_HELD": self.held,
                "DI_FULL": full,
                "DI_READY": self.ready,
            })
        except Exception as e:
            self.get_logger().warn("写 PLC 状态失败：%s" % e, throttle_duration_sec=5.0)

    def shutdown(self):
        self.io.close()


def main():
    rclpy.init()
    node = PlcGateway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
