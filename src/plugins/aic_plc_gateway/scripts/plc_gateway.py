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

        p = lambda n: self.get_parameter(n).value
        self.allow_control = bool(p("allow_control"))
        self.n_mat = int(p("n_materials"))
        self.store = DataStore()

        # ---- 内部状态 ----
        self.cam_state = CAM_IDLE
        self.cam_t0 = None
        self.cam_slot = 0
        self.axis_state = AX_IDLE
        self.task_state = TK_IDLE
        self.staged = 0          # 置物台上的物料号，0=空
        self.cam_type = 0        # 最近一次识别结果 1..4，0=未识别
        self.placed_mask = 0     # bit(n-1) = 第 n 号已入位
        self.placed = 0
        self.goto_cnt = 0
        self.visit = 0
        self.err = ERR_NONE
        self.held = False
        self.ready = False
        self.actual = {"x": 0.0, "y": 0.0, "z": 0.0}

        self._last_cmd = 0
        self._last_trigger = False
        self._last_attach = False
        self._last_detach = False
        self._last_reset = False
        self._self_pub_until = 0.0
        self._warned_observe = False
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

        # ---- Modbus 从站 ----
        self.slave = ModbusTcpSlave(self.store, str(p("host")), int(p("port")),
                                    int(p("unit_id")), self._slave_log)
        try:
            self.slave.start()
        except OSError as e:
            self.get_logger().error(
                "Modbus 端口 %d 绑定失败：%s（被占用？低于 1024 需 root）"
                % (int(p("port")), e))
            raise

        self.create_timer(1.0 / max(5.0, float(p("poll_hz"))), self._tick)

        self.get_logger().info(
            "PLC 网关就绪：modbus tcp %s:%d unit=%d，模式=%s"
            % (p("host"), int(p("port")), int(p("unit_id")),
               "接管" if self.allow_control else "观察（只读，可与 bt_runner 共存）"))

    # ---- 内部 ----
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
        with self.store.lock:
            self.store.holding[CAM_DATA] = tid & 0xFFFF
            self.store.holding[CAM_MS] = int((_now() - self.cam_t0) * 1000) if self.cam_t0 else 0
            self.store.holding[CAM_ERR] = 0

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
        with self.store.lock:
            task_cmd = self.store.holding[TASK_CMD]
            pos = self.store.holding[POS_SET]
            cam_req = self.store.holding[CAM_REQ]
            trig = self.store.coils[COIL_TRIGGER]
            att = self.store.coils[COIL_ATTACH]
            det = self.store.coils[COIL_DETACH]
            rst = self.store.coils[COIL_RESET]

        # 任务：先写 POS_SET 设定目标，再写 TASK_CMD 触发（PLC 的「设定值 + 执行位」）
        if task_cmd and task_cmd != self._last_cmd:
            self._exec_task(task_cmd, pos)
            with self.store.lock:
                self.store.holding[TASK_CMD] = 0
        self._last_cmd = task_cmd

        if cam_req and self.allow_control:
            with self.store.lock:
                self.store.holding[CAM_REQ] = 0
            self.trigger_pub.publish(Empty())
            self.cam_state = CAM_BUSY
            self.cam_t0 = _now()
            self.get_logger().info("PLC 请求拍照 -> 已触发 vision")

        if trig and not self._last_trigger:
            self.trigger_pub.publish(Empty())
            with self.store.lock:
                self.store.coils[COIL_TRIGGER] = False
        self._last_trigger = trig

        if att and not self._last_attach:
            self._exec_attach()
            with self.store.lock:
                self.store.coils[COIL_ATTACH] = False
        self._last_attach = att

        if det and not self._last_detach:
            self._exec_detach()
            with self.store.lock:
                self.store.coils[COIL_DETACH] = False
        self._last_detach = det

        if rst and not self._last_reset:
            self.placed = 0
            self.placed_mask = 0
            self.staged = 0
            self.held = False
            self.err = ERR_NONE
            self.task_state = TK_IDLE
            self.cam_state = CAM_IDLE
            with self.store.lock:
                self.store.coils[COIL_RESET] = False
            self.get_logger().info("PLC 复位")
        self._last_reset = rst

    def _exec_task(self, cmd, slot):
        """PLC 动作：1=GOTO 2=HANDLE。"""
        if cmd not in (1, 2):
            self._set_err(ERR_NONE, "未知 TASK_CMD=%d，忽略" % cmd)
            return
        if not self.allow_control:
            if not self._warned_observe:
                self.get_logger().warn(
                    "观察模式收到 TASK_CMD=%d，已忽略。要由 PLC 下发任务请用 "
                    "allow_control:=true（并停掉 bt_runner）" % cmd)
                self._warned_observe = True
            self._set_err(ERR_OBSERVE, "观察模式不接受控制写入")
            return
        if not 1 <= slot <= self.n_mat:
            self._set_err(ERR_NO_SLOT, "Pos=%d 越界（应 1..%d）" % (slot, self.n_mat))
            return
        # HANDLE 必须带类型：控制器按 material_type 决定放进哪一列。
        # 不带（或 0）会走「未识别」快速分支——不动、不入架，但照样回 handled，
        # 于是 PLC 侧看到 TASK_STATE=DONE 却什么都没放进去（这个坑很隐蔽）。
        if cmd == 2 and self.cam_type not in (1, 2, 3, 4):
            self._set_err(ERR_NO_TYPE, "HANDLE 时无有效识别结果（CAM_DATA=%d）"
                          % self.cam_type)
            return
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

    def _exec_attach(self):
        if not self.allow_control:
            self.get_logger().warn("观察模式忽略 COIL_ATTACH", throttle_duration_sec=5.0)
            return
        if not 1 <= self.staged <= self.n_mat:
            self._set_err(ERR_NO_STAGED, "台面无料，无法吸附")
            return
        self.attach_pub[self.staged].publish(Empty())
        self.held = True
        self.axis_state = AX_MOVING
        self.get_logger().info("PLC -> attach material_%02d" % self.staged)

    def _exec_detach(self):
        if not self.allow_control:
            self.get_logger().warn("观察模式忽略 COIL_DETACH", throttle_duration_sec=5.0)
            return
        if not 1 <= self.staged <= self.n_mat:
            self._set_err(ERR_NO_STAGED, "台面无料，无法释放")
            return
        self.detach_pub[self.staged].publish(Empty())
        self.held = False
        self.axis_state = AX_MOVING
        self.get_logger().info("PLC -> detach material_%02d" % self.staged)

    def _publish_status(self):
        now = _now()
        if now - self._hb_t >= 1.0:
            self._hb = (self._hb + 1) & 0xFFFF
            self._hb_t = now
        full = (self.placed_mask & ((1 << self.n_mat) - 1)) == ((1 << self.n_mat) - 1)
        with self.store.lock:
            h = self.store.holding
            h[HEARTBEAT] = self._hb
            h[CAM_STATE] = self.cam_state
            h[X_ACT] = to_int16_mm(self.actual["x"]) & 0xFFFF
            h[Y_ACT] = to_int16_mm(self.actual["y"]) & 0xFFFF
            h[Z_ACT] = to_int16_mm(self.actual["z"]) & 0xFFFF
            h[AXIS_STATE] = self.axis_state
            h[SHELF_BITS] = self.placed_mask & 0xFFFF
            h[SHELF_FULL] = 1 if full else 0
            h[PLACED_CNT] = self.placed & 0xFFFF
            h[TASK_STATE] = self.task_state
            h[ERR] = self.err
            d = self.store.discrete
            d[DI_INPOS] = (self.axis_state == AX_INPOS)
            d[DI_CAM_DONE] = (self.cam_state == CAM_DONE)
            d[DI_HELD] = self.held
            d[DI_FULL] = full
            d[DI_READY] = self.ready

    def shutdown(self):
        self.slave.stop()


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
