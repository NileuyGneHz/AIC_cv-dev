#!/usr/bin/python3
"""三轴龙门控制侧仿真实现（M3：对齐官方平台流程）。

契约：docs/guides/decision_control_interface.md（decision -> control）
      实际消息定义以 src/plugins/aic_msgs/msg/MaterialTask.msg 为准（uint8 + action 字符串）。

M3 轮次语义（官方 VirtualNext 平台：置物台上料 -> 拍照 -> 吸盘取 -> 放货架列位）
--------
* GOTO(slot)：第 k 个 GOTO = 第 k 轮。先把 material_k 从来料架格位传送到置物台
  （ign service set_pose；上一轮识别失败留在台面的物料先送回收位），
  然后龙门走到置物台上方，到位发 /sim/gantry_arrived -> 视觉识别。
* HANDLE(slot, type)：吸盘下探吸附（attach material_k）-> 提升 -> 平移到
  货架「类型 type 列」的最低空层 -> 放入（每格一块，不堆叠）-> 松开 -> 提升，
  发 /sim/handle_done。slot 在 M3 只是轮次编号，不再对应取料库位。
* 决策侧零改动：话题契约 / 状态机不变。

实现要点
--------
* 指令话题是 fire-and-forget：本节点维护 FIFO 队列，逐条执行，不抢决策的节奏。
* 运动用「梯形速度剖面 + 每 20 ms 流式下发 setpoint」驱动 JointPositionController
  （/aic/joint/{x,y,z}/cmd_pos），x/y/z 协调运动（同起同停）。
* 到位判定用「剖面走完 + 关节反馈收敛」（position/velocity 双阈值 + 连续 N 拍）；
  place 步用「触底判据」（M2 坑 15）。
* 吸盘 = DetachableJoint 话题对 /sim/attach_NN、/sim/detach_NN（NN = 物料编号），
  语义对应官方的 SuctionEnable / 关闭吸盘。

参数（launch 里统一给）
----------------------
  layout_yaml       默认空 = 包内 config/sim_layout.yaml
  task_topic        /control/material_task
  arrived_topic     /sim/gantry_arrived
  handled_topic     /sim/handle_done
  trigger_topic     /vision/trigger
  joint_state_topic /world/aic_warehouse/model/aic_gantry/joint_state
  xy_speed          0.30 m/s   z_speed 0.10 m/s
  pos_tol/vel_tol   到位阈值（默认取 sim_layout.yaml 的 gantry 段）
  settle_s          0.15 s（仅在拿不到关节反馈时退化成定时等待）
  handle_dwell_s    0.35 s（抓取停留，夹爪动作占位）
  release_rounds    4（开机广播几轮 detach，把 12 块铁片从 z_carriage 上放开）
  release_interval_s 0.4 s
  place_tol         0.015 m（放到码放格时允许的「触底静差」，来自同类型叠放）
"""
import collections
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, UInt8

from aic_msgs.msg import MaterialTask
from sim_layout import SimLayout, declare_number

# 关节软限位，与 models/aic_gantry/model.sdf 的 <limit> 保持一致
# M3：z 上限 +0.03 —— 携料平移时盘底必须跨过第 3 层围边顶（0.232）；
#     下限 -0.20 覆盖最低取放位（置物台盘顶 0.106 → z=-0.133）
LIMITS = {
    "x": (-0.45, 0.45),
    "y": (-0.24, 0.32),
    "z": (-0.20, 0.03),
}
AXES = ("x", "y", "z")


def _clamp(v, lo_hi):
    return max(lo_hi[0], min(lo_hi[1], v))


class _Step:
    """一步动作：move（协调梯形剖面）/ dwell（停留）/ event（发通知）。"""

    def __init__(self, kind, **kw):
        self.kind = kind
        self.__dict__.update(kw)


def _profile(u, ramp=0.2):
    """归一化梯形速度剖面的位移比 f(u)，f(0)=0, f(1)=1。"""
    if u <= 0.0:
        return 0.0
    if u >= 1.0:
        return 1.0
    a = ramp
    norm = 1.0 - a  # 面积（速度曲线下面积）
    if u < a:
        return (u * u) / (2.0 * a) / norm
    if u <= 1.0 - a:
        return (u - a / 2.0) / norm
    return 1.0 - ((1.0 - u) * (1.0 - u)) / (2.0 * a) / norm


class GantryController(Node):
    def __init__(self):
        super().__init__("gantry_controller")

        self.layout = SimLayout.load(self.declare_parameter("layout_yaml", "").value)
        task_topic = self.declare_parameter("task_topic", "/control/material_task").value
        arrived_topic = self.declare_parameter("arrived_topic", "/sim/gantry_arrived").value
        handled_topic = self.declare_parameter("handled_topic", "/sim/handle_done").value
        # 速度优先取 sim_layout.yaml（唯一数据源），launch 参数可覆盖
        self.xy_speed = declare_number(
            self, "xy_speed", float(self.layout.gantry.get("xy_speed", 0.30)))
        self.z_speed = declare_number(
            self, "z_speed", float(self.layout.gantry.get("z_speed", 0.20)))
        self.settle_s = declare_number(self, "settle_s", 0.15)
        self.dwell_s = declare_number(self, "handle_dwell_s", 0.35)
        self.travel_z = float(self.layout.gantry.get("travel_z", 0.0))
        # 吸盘工具基准（z_joint=0 时吸口下沿世界高度，与 model.sdf 一致）
        self.pad_z0 = self.layout.pad_z0()
        self.plate_th = float(self.layout.plate["thickness"])
        # 「贴住盘顶」的关节值 = 盘顶高度 - pad_z0
        self.pick_z = float(self.layout.staging["z"]) + self.plate_th - self.pad_z0
        # 到位判据：位置阈值来自布局文件（与真机调参同一处），速度阈值默认 10 mm/s
        self.pos_tol = declare_number(self, "pos_tol", float(self.layout.gantry.get("pos_tol", 0.003)))
        self.vel_tol = declare_number(self, "vel_tol", float(self.layout.gantry.get("vel_tol", 0.010)))
        self.confirm_ticks = int(declare_number(self, "confirm_ticks", 4))
        # place 步（放到码放格）的「触底」判据：落点会叠放，允许几毫米静差
        self.place_tol = declare_number(self, "place_tol", 0.015)
        self.place_vel_tol = declare_number(self, "place_vel_tol", 0.003)
        self.move_timeout_s = declare_number(
            self, "move_timeout_s", float(self.layout.gantry.get("move_timeout_s", 8.0)))
        rate = declare_number(self, "control_rate_hz", 50.0)
        self.dt = 1.0 / rate

        self.pub = {a: self.create_publisher(Float64, "/aic/joint/{}/cmd_pos".format(a), 10)
                    for a in AXES}
        # 抓取/释放信号：每块铁片一个 DetachableJoint 话题（由 gen_world.py 生成插件时定名）。
        # 都是 std_msgs/Empty，经 ros_gz_bridge 转成 ignition.msgs.Empty。
        # 坑：话题名不能带「以数字开头的 token」——/sim/attach/01 会被 rclpy 拒
        #     （InvalidTopicNameException，节点启动即崩），所以编号直接接在词尾。
        n_slots = self.layout.cols * self.layout.rows
        self.attach_pub = {i: self.create_publisher(Empty, "/sim/attach_{:02d}".format(i), 10)
                           for i in range(1, n_slots + 1)}
        self.detach_pub = {i: self.create_publisher(Empty, "/sim/detach_{:02d}".format(i), 10)
                           for i in range(1, n_slots + 1)}
        self.arrived_pub = self.create_publisher(UInt8, arrived_topic, 10)
        self.handled_pub = self.create_publisher(UInt8, handled_topic, 10)
        # 到位即发采集请求：真视觉节点（aic_vision_trad）订阅这条，与决策侧接口约定一致
        # （/vision/trigger，见 docs/guides/decision_vision_interface.md 第 1 节）。
        trigger_topic = self.declare_parameter("trigger_topic", "/vision/trigger").value
        self.trigger_pub = self.create_publisher(Empty, trigger_topic, 10)
        self.create_subscription(MaterialTask, task_topic, self._on_task, 10)

        # 就绪握手：decision 侧 control_wait_ready=true 时等这条 true 才开跑。
        # 必须周期重发——决策端是 volatile 订阅，只发一次会被后启动的节点错过。
        # 注意：首次发送放在状态初始化之后（_publish_ready 依赖 _release_done/_t0）。
        ready_topic = self.declare_parameter("ready_topic", "/control/ready").value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, ready_topic, qos)

        # 理论位置（我们自己的 setpoint 账本）
        self.pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        # 实际位置/速度（来自 Gazebo 关节反馈；None = 还没收到，退回定时等待）
        self.actual = {a: None for a in AXES}
        self.vel = {a: 0.0 for a in AXES}
        self.queue = collections.deque()
        self.steps = collections.deque()   # 当前任务剩余步骤
        self.active = None                 # 正在执行的 move/dwell
        self.busy = False

        # 开机松爪：DetachableJoint 在首次 PreUpdate 就把 child 焊到 parent_link 上
        # （Fortress 版没有 initial_attach 之类的开关，实测插件只认 parent_link /
        #  child_model / child_link / attach_topic / detach_topic / output_topic 六个字段）。
        # 12 个插件 = 12 块铁片一开始全挂在 z_carriage 上：静置 z 被压到 -3.675 mm，
        # 正好等于 12*0.05kg*9.81 / p_gain(1600)，且速度恒为 0 —— 闭环判据永远等不到。
        # 对策：开机后广播几轮 detach 把它们放回台面。
        # 两条约束：
        #   1) 通知 /control/ready 必须等松爪跑完再发。实测决策端在控制器启动后
        #      0.02 s 就发来第一条 GOTO，若只按「收到任务就停发」来保护，松爪会被
        #      永久跳过 —— 12 块铁片继续焊在 z_carriage 上，Z 轴 8 s 只走 0.2 mm
        #      （日志：目标 -0.04 / 实际 -0.0002 / 误差 39.8 mm）。
        #      而「松爪要不要开始」用第一帧 joint_state 判定：它同时证明
        #      ros_gz_bridge 已经起来、ROS->GZ 方向的 24 个 gz 订阅也已建好
        #      （否则 detach 消息全丢）。兜底：等 release_wait_s 还没反馈也照发。
        #   2) 松爪轮次跳过 _held_slot，防止把刚夹起的料中途放掉。
        self._release_left = int(declare_number(self, "release_rounds", 4))
        self._release_period = declare_number(self, "release_interval_s", 0.4)
        self._release_wait_s = declare_number(self, "release_wait_s", 3.0)
        self._release_next = 0.0     # time.monotonic() 到期时间；0 = 立刻
        self._release_done = False
        self._t0 = time.monotonic()
        # 当前夹着的槽位；松爪轮次跳过它，防止把刚夹起的料中途放掉
        self._held_slot = 0
        # 关节反馈看门狗（见 _watch_sim_alive）
        self._js_last = None
        self._js_warn_next = 0.0

        # ---- M3 轮次状态 ----
        # 轮次 = 已收到的 GOTO 数；第 k 轮处理的物料 = material_k（生成顺序即真值序）
        self._round = 0
        self._staged = 0        # 当前在置物台上的物料编号（0 = 空）
        self._picked = False    # 置物台上的物料是否已被吸盘取走
        self._placed = collections.Counter()   # 每个类型列已放的块数（选最低空层）
        # 置物台坐标缓存 + GOTO 停靠偏移（吸盘让出固定相机的光路）
        self._staging_xy = self.layout.staging_xy()
        self._goto_offset_y = float(
            self.layout.staging.get("goto_offset_y", 0.10))

        # 关节反馈订阅。桥在 sim.launch.py 里（ignition.msgs.Model -> sensor_msgs/JointState）。
        # QoS 用 BEST_EFFORT：gz 侧按 sensor data 发，reliable 订阅者会「静默收不到」。
        js_topic = self.declare_parameter(
            "joint_state_topic", "/world/aic_warehouse/model/aic_gantry/joint_state").value
        js_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, js_topic, self._on_joint_state, js_qos)

        self.create_timer(self.dt, self._tick)
        # 现在就报一次 ready（若松爪还没跑完，_publish_ready 会自己压住，1 Hz 重试）
        self._publish_ready()
        self.create_timer(1.0, self._publish_ready)
        self._publish_all()
        self.get_logger().info(
            "[sim/control] 龙门控制就绪：topic='%s'，布局 %s" % (task_topic, self.layout.summary()))

    # ------------------------------------------------------------------ 任务
    def _on_joint_state(self, msg):
        """关节反馈：x_joint / y_joint / z_joint -> actual/vel（单位 m、m/s）。"""
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            axis = name.split("_")[0]
            if axis in self.actual:
                self.actual[axis] = float(pos)
                self.vel[axis] = float(vel)
        self._js_last = time.monotonic()

    def _watch_sim_alive(self):
        """关节反馈断流告警：仿真停住时，下游看到的是「到位超时 + 视觉超时」。

        坑：ogre2 无头渲染 + 12 个 DetachableJoint 长时间跑，实测偶发整条
        run loop 卡死（gzserver 进程还在、无任何报错，但 joint_state 不再更新、
        相机也不再出帧）。没有这条告警的话，现场会误判成「视觉挂了 / 控制器没发指令」。
        自检顺序：先看这条 warn，再看 `ign topic -e -t /clock` 时间戳走不走。
        """
        if self._js_last is None:
            return
        silent = time.monotonic() - self._js_last
        if silent < 2.0 or silent < self._js_warn_next:
            return
        self._js_warn_next = silent + 10.0
        self.get_logger().error(
            "[sim/control] 关节反馈已断流 %.1f s（仿真可能已停住，不是控制/视觉问题）；"
            "自检：ign topic -e -t /clock 看时间戳是否还在走" % silent)

    def _publish_ready(self):
        # 等开机松爪跑完再报 ready（原因见 __init__ 里的注释）。10 s 硬兜底：
        # 万一关节反馈一直不来，也不能把决策端卡死（bt_runner 的等待上限也是 10 s）。
        if not self._release_done and time.monotonic() - self._t0 < 10.0:
            return
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)

    def _maybe_release(self):
        """开机把全部 DetachableJoint 松开（原因见 __init__ 里的注释）。"""
        if self._release_done:
            return
        now = time.monotonic()
        if self.actual["x"] is None and (now - self._t0) < self._release_wait_s:
            return                      # 桥还没通，发了也是白发
        if self._release_next and now < self._release_next:
            return
        self._release_next = now + self._release_period
        self._release_left -= 1
        for slot, pub in self.detach_pub.items():
            if slot != self._held_slot:
                pub.publish(Empty())
        if self._release_left <= 0:
            self._release_done = True
            self.get_logger().info("[sim/control] 开机松爪完成：%d 块铁片已全部释放" %
                                   len(self.detach_pub))
            self._publish_ready()      # 别等 1 Hz 那拍，立刻放行决策端

    def _on_task(self, msg):
        action = (msg.action or "").strip().upper()
        slot = int(msg.slot_id)
        kind = int(msg.material_type)
        self.get_logger().info(
            "[sim/control] 收到任务 %s slot=%d type=%d visit=%d"
            % (action, slot, kind, msg.visit_index))

        if action == "GOTO":
            self._round += 1
            # 轮次随步骤一起入队：队列可能积压（决策 fire-and-forget），
            # 执行时再读 self._round 会把第 1 轮的日志打成「轮次 2」。
            self.queue.append(self._goto_steps(slot, self._round))
        elif action == "HANDLE":
            self.queue.append(self._handle_steps(slot, kind))
        elif action == "HOME":
            self.queue.append(self._home_steps())
        else:
            self.get_logger().warn("[sim/control] 未知 action='%s'，忽略" % msg.action)

    def _goto_steps(self, slot, rnd):
        """M4：GOTO = 去置物台上方（到位后触发视觉识别置物台上的物料）。

        供料按 slot_id：第 slot 号 GOTO 把 material_slot 从**来料架**它自己的
        格位传送到置物台（来料架 3 行 x 4 列，格位与物料编号一一对应）。
        上一轮若识别失败（物料还留在置物台），先把它传送到回收位，
        保证「台面同时只有一块」。
        """
        sx, sy, _sz = self._staging_xy
        steps = []
        # 回收上轮遗留（识别失败未取走的物料）
        if self._staged and not self._picked:
            steps.append(_Step("signal", name="recycle", material=self._staged))
            self.get_logger().warn(
                "[sim/control] material %02d 未被取走，送回收位（决策侧该位点已 TIMEOUT）"
                % self._staged)
        # 上料：material_slot -> 置物台
        n_total = self.layout.cols * self.layout.rows
        if 1 <= slot <= n_total:
            steps.append(_Step("signal", name="spawn", material=slot))
            self._staged = slot
            self._picked = False
        else:
            self._staged = 0
            self.get_logger().error(
                "[sim/control] slot %d 越界（1..%d），不上料" % (slot, n_total))
        if self.pos["z"] < self.travel_z - 1e-4:
            steps.append(_Step("move", targets={"z": self.travel_z}, speed=self.z_speed))
        # 停靠点 = 置物台 + offset：相机（固定在置物台正上方）拍完整盘面，
        # 吸盘停在旁边不进光路；HANDLE 再平移到正上方下探
        steps.append(_Step("move", targets={"x": sx, "y": sy + self._goto_offset_y},
                           speed=self.xy_speed))
        steps.append(_Step("settle", seconds=self.settle_s))
        steps.append(_Step("event", name="arrived", slot=slot, round=rnd))
        return steps

    def _handle_steps(self, slot, mtype):
        """吸盘取料 -> 搬到该类型列的最低空层 -> 放入（每格一块，不堆叠）。

        类型未知（0）时不下探不吸附（对应决策 SkipSlot：未识别不放），
        物料留在置物台，下一轮 GOTO 会把它送回收位。
        """
        if mtype not in (1, 2, 3, 4) or self._staged == 0:
            # 未识别：不平移不下探（物料留在置物台，下一轮 GOTO 送回收位）
            steps = [_Step("move", targets={"z": self.travel_z}, speed=self.z_speed),
                     _Step("event", name="handled", slot=slot)]
            return steps
        if self._staged != slot:
            # 协议错乱（HANDLE 与台面物料不是同一块）：宁可报错也别把错的料吸走。
            # 正常链路不会出现——决策严格按 GOTO(N) -> 视觉 -> HANDLE(N) 下发。
            self.get_logger().error(
                "[sim/control] HANDLE slot=%d，但置物台上是 material %02d："
                "按台面物料继续（请检查决策/控制时序）" % (slot, self._staged))
        level = self._placed[mtype] + 1
        cell = self.layout.rack_cell(mtype, level)
        steps = []
        if cell is None:
            self.get_logger().error(
                "[sim/control] 类型 %d 的层位已满（%d/%d），不放置"
                % (mtype, self._placed[mtype], self.layout.rows))
            steps.append(_Step("move", targets={"z": self.travel_z}, speed=self.z_speed))
            steps.append(_Step("event", name="handled", slot=slot))
            return steps
        cx, cy, deck_z = cell
        # 放置目标：吸口贴住放到托盘上的盘顶
        place_z = deck_z + self.plate_th - self.pad_z0
        self.get_logger().info(
            "[sim/control] material %02d（type=%d）-> 货架列 %d 第 %d 层 (%.3f, %.3f, %.3f)"
            % (slot, int(mtype), mtype, level, cx, cy, deck_z))
        steps += [
            # 从停靠偏移位平移到置物台正上方，再下探吸附
            _Step("move", targets={"x": self._staging_xy[0],
                                   "y": self._staging_xy[1]}, speed=self.xy_speed),
            _Step("move", targets={"z": self.pick_z}, speed=self.z_speed),
            _Step("dwell", seconds=self.dwell_s),
            _Step("signal", name="attach", material=self._staged),
            _Step("move", targets={"z": self.travel_z}, speed=self.z_speed),
            _Step("move", targets={"x": cx, "y": cy}, speed=self.xy_speed),
            # 放置用 place 步（触底判据）：托盘面接触后位置控制器还在压，
            # 按精确到位判会白等到 move_timeout_s（M2 实测过的坑）
            _Step("place", targets={"z": place_z}, speed=self.z_speed),
            _Step("dwell", seconds=self.dwell_s),
            _Step("signal", name="detach", material=self._staged),
            _Step("move", targets={"z": self.travel_z}, speed=self.z_speed),
            _Step("event", name="handled", slot=slot),
        ]
        # 层位与「已取走」必须在**生成步骤时**提交（别改成 detach 时提交）：
        # 决策是 fire-and-forget——DispatchMaterialTask 之后立刻 RecordSlotResult 记 OK，
        # 不等 handle_done 就发下一轮 GOTO。队列里可能同时压着 GOTO(N)、HANDLE(N)、
        # GOTO(N+1)。若等 detach 执行时才提交：
        #   1) 同类型的下一块 HANDLE 会用同一个 level，两块抢同一层；
        #   2) GOTO(N+1) 生成时 `_picked` 仍为 False，会把已安排取走的物料误送回收位
        #      （实测掉料 1/12，material_04 落到回收位 z=0.023）。
        self._placed[mtype] = level
        self._picked = True
        return steps

    def _home_steps(self):
        return [
            _Step("move", targets={"z": self.travel_z}, speed=self.z_speed),
            _Step("move", targets={"x": 0.0, "y": 0.0}, speed=self.xy_speed),
            _Step("event", name="home", slot=0),
        ]

    # ------------------------------------------------------------------ 时钟
    def _publish_all(self):
        """把三个轴的目标位置都发一遍。

        必须每拍都发（不只是运动时）：Z 轴是竖直 prismatic，控制器是速度伺服，
        一旦停发，重力会把 Z 滑到下极限（实测 z_carriage 从 0.545 掉到 0.125，
        相机直接掉到地面以下）。持续下发 = 位置保持。
        """
        for axis in AXES:
            self._publish(axis)

    def _tick(self):
        self._maybe_release()
        self._watch_sim_alive()
        if self.active is None:
            if not self.steps:
                if not self.queue:
                    self._publish_all()
                    return
                self.steps.extend(self.queue.popleft())
                if not self.busy:
                    self.busy = True
            self._start_next()
            if self.active is None:
                self._publish_all()
                return
        self._advance()
        self._publish_all()

    def _start_next(self):
        while self.steps and self.active is None:
            step = self.steps.popleft()
            if step.kind in ("event", "signal"):
                self._emit(step)
                continue
            self.active = step
            if step.kind in ("dwell", "settle"):
                step.remaining = step.seconds
            else:  # move
                step.start = dict(self.pos)
                dist = 0.0
                for a, target in step.targets.items():
                    target = _clamp(float(target), LIMITS[a])
                    step.targets[a] = target
                    dist = max(dist, abs(target - self.pos[a]))
                # 至少 0.12 s，避免零位移时除零/瞬移
                step.duration = max(0.12, dist / max(step.speed, 1e-3))
                step.elapsed = 0.0
                self.get_logger().info(
                    "[sim/control] 运动 -> %s（%.3f s）"
                    % ({a: round(v, 4) for a, v in step.targets.items()}, step.duration))

    def _advance(self):
        step = self.active
        dt = self.dt
        if step.kind in ("dwell", "settle"):
            step.remaining -= dt
            if step.remaining <= 0.0:
                self.active = None
            return

        # 剖面走完后进入「保持 + 等实际收敛」阶段
        if getattr(step, "phase", "profile") == "holding":
            self._hold_until_settled(step)
            return

        step.elapsed += dt
        u = min(1.0, step.elapsed / step.duration)
        f = _profile(u)
        for a, target in step.targets.items():
            self.pos[a] = step.start[a] + (target - step.start[a]) * f
            self._publish(a)
        # 未列出的轴保持原值，但也要持续下发，防止控制器被别处覆盖
        for a in AXES:
            if a not in step.targets:
                self._publish(a)
        if u >= 1.0:
            for a, target in step.targets.items():
                self.pos[a] = target
                self._publish(a)
            step.phase = "holding"
            step.hold_s = 0.0
            step.confirm = 0

    def _hold_until_settled(self, step):
        """保持 setpoint，等实际位置/速度都进阈值；超时则告警放行。"""
        step.hold_s += self.dt
        # 保持阶段必须继续下发：Z 轴是竖直 prismatic，停发会被重力拖下去
        self._publish_all()

        ok, detail = self._converged(step)
        step.confirm = step.confirm + 1 if ok else 0
        if step.confirm >= self.confirm_ticks:
            self.get_logger().info(
                "[sim/control] 到位 %s（%s，剖面后 %.2f s）"
                % ({a: round(v, 4) for a, v in self.pos.items()}, detail, step.hold_s))
            self.active = None
            return
        # place 步：触底即算完成。allowed 的静差只来自「铁片叠放」，
        # 不能给太大，否则会把「压不下去」误判成「已放下」。
        if step.kind == "place" and step.hold_s >= 0.30 and self._touched_down(step):
            self.get_logger().info(
                "[sim/control] 已触底 %s（%s，剖面后 %.2f s）"
                % ({a: round(v, 4) for a, v in step.targets.items()},
                   self._place_detail(step), step.hold_s))
            self.active = None
            return
        if step.hold_s >= self.move_timeout_s:
            self.get_logger().warn(
                "[sim/control] 到位超时 %.1f s（%s），按到位继续；目标 %s 实际 %s"
                % (step.hold_s, detail,
                   {a: round(v, 4) for a, v in step.targets.items()},
                   {a: (None if self.actual[a] is None else round(self.actual[a], 4))
                    for a in step.targets}))
            self.active = None

    def _axis_error(self, step):
        """step.targets 里各轴的最大位置误差与最大残余速度（米 / 米每秒）。"""
        worst_pos = 0.0
        worst_vel = 0.0
        for a in step.targets:
            worst_pos = max(worst_pos, abs(self.actual[a] - step.targets[a]))
            worst_vel = max(worst_vel, abs(self.vel[a]))
        return worst_pos, worst_vel

    def _touched_down(self, step):
        """place 步是否已触底：位置压在目标下方一点点且不再动。

        判定要同时看「速度接近 0」和「误差在 place_tol 内」：
        只看误差会把「正常下探途中恰好经过该区间」当成触底。
        """
        if any(self.actual[a] is None for a in AXES):
            return False
        worst_pos, worst_vel = self._axis_error(step)
        return worst_pos <= self.place_tol and worst_vel <= self.place_vel_tol

    def _place_detail(self, step):
        worst_pos, worst_vel = self._axis_error(step)
        return "静差 %.1f mm，残余速度 %.1f mm/s" % (worst_pos * 1000.0, worst_vel * 1000.0)

    def _converged(self, step):
        """判断 step.targets 里的轴是否已真正停稳。返回 (bool, 说明)。"""
        if any(self.actual[a] is None for a in AXES):
            return step.hold_s >= self.settle_s, "无关节反馈，按 %.2f s 定时等待" % self.settle_s
        worst_pos, worst_vel = self._axis_error(step)
        return (worst_pos <= self.pos_tol and worst_vel <= self.vel_tol,
                "最大误差 %.1f mm，最大速度 %.1f mm/s"
                % (worst_pos * 1000.0, worst_vel * 1000.0))

    def _publish(self, axis):
        msg = Float64()
        msg.data = float(self.pos[axis])
        self.pub[axis].publish(msg)

    def _set_pose(self, model, x, y, z):
        """用 ign service 把模型传送到指定位姿（上料 / 回收）。

        为什么不用动态 create 实体：见 sim_layout.yaml infeed 段的注释 ——
        DetachableJoint 在世界加载时解析 child_model，后创建的实体绑不上。

        三个必踩的坑（都是实测）：
        1. 请求类型是 ignition.msgs.Pose（扁平字段），**不是** SetPose ——
           `ignition.msgs.SetPose` 在 Fortress 里不存在，CLI 会报
           "Unable to create request"，而节点里只看 rc 的话会当成网络问题。
        2. 服务由 UserCommands 插件提供。世界没加载它时服务压根不存在，
           客户端不报「无此服务」而是等满 timeout 报 "Service call timed out"，
           极易误判成「动态模型不能 set_pose」。
        3. 动态模型若仍被 DetachableJoint 焊在龙门上，set_pose 返回 true 但
           位置不动（约束每帧把它拉回）。控制器开机已广播 12 路 detach，
           所以这里能生效；单独调试时记得先发 /sim/detach_NN。

        必须用 subprocess 继承本进程环境（含 IGN_PARTITION），否则并行
        隔离时会把命令发到别的分区去。
        """
        req = ('name: "{m}" position: {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} '
               'orientation: {{w: 1.0}}').format(m=model, x=x, y=y, z=z)
        try:
            r = subprocess.run(
                ["ign", "service", "-s", "/world/aic_warehouse/set_pose",
                 "--reqtype", "ignition.msgs.Pose",
                 "--reptype", "ignition.msgs.Boolean",
                 "--timeout", "2000", "--req", req],
                capture_output=True, text=True, timeout=6.0)
            if r.returncode != 0 or "data: true" not in (r.stdout or ""):
                self.get_logger().error(
                    "[sim/control] set_pose %s 失败 rc=%d：%s"
                    % (model, r.returncode,
                       (r.stderr or r.stdout).strip()[-200:]))
                return False
            return True
        except Exception as exc:  # noqa: BLE001 - 传送失败只影响该轮，不拖垮控制器
            self.get_logger().error("[sim/control] set_pose %s 异常：%s" % (model, exc))
            return False

    def _emit(self, step):
        if step.name == "attach":
            mat = int(step.material)
            self._held_slot = mat   # 沿用 M2 字段名：松爪轮次跳过它
            self.attach_pub[mat].publish(Empty())
            self.get_logger().info(
                "[sim/control] material %02d 吸附（吸盘使能 / DetachableJoint attach）" % mat)
        elif step.name == "detach":
            mat = int(step.material)
            self.detach_pub[mat].publish(Empty())
            if self._held_slot == mat:
                self._held_slot = 0
            self.get_logger().info(
                "[sim/control] material %02d 释放（吸盘断气 / DetachableJoint detach）" % mat)
        elif step.name == "spawn":
            mat = int(step.material)
            sx, sy, sz = self._staging_xy
            ok = self._set_pose(
                "material_{:02d}".format(mat), sx, sy,
                sz + self.plate_th / 2.0 + 0.002)
            self.get_logger().info(
                "[sim/control] material %02d 上料 -> 置物台 %s"
                % (mat, "OK" if ok else "失败（该轮视觉将超时）"))
        elif step.name == "recycle":
            mat = int(step.material)
            rx, ry, rz = self.layout.recycle_xyz()
            ok = self._set_pose("material_{:02d}".format(mat), rx, ry,
                                rz + self.plate_th / 2.0 + 0.001)
            self.get_logger().info(
                "[sim/control] material %02d -> 回收位 %s" % (mat, "OK" if ok else "失败"))
            if self._staged == mat:
                self._staged = 0
        elif step.name == "arrived":
            msg = UInt8()
            msg.data = int(step.slot)
            self.arrived_pub.publish(msg)
            # 采集请求：真视觉链路靠这条触发拍照（ground-truth 桩不订阅它，无副作用）
            self.trigger_pub.publish(Empty())
            self.get_logger().info("[sim/control] 轮次 %d 到位置物台上方，已通知 vision"
                                   % int(getattr(step, "round", self._round)))
        elif step.name == "handled":
            msg = UInt8()
            msg.data = int(step.slot)
            self.handled_pub.publish(msg)
            self.get_logger().info("[sim/control] slot %d 处理完成" % step.slot)
        elif step.name == "home":
            self.get_logger().info("[sim/control] 已回原点")


def main():
    rclpy.init()
    node = GantryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # 关停时 spin 抛这个（继承 RuntimeError，但 rclpy.ok() 仍为真）——
        # 不接住就会 exit 1，launch 报 process has died，录像/自验时看着像真故障。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
