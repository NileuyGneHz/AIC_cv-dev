#!/usr/bin/python3
"""三轴龙门控制侧仿真实现（v1）：吃 /control/material_task，驱动机台走位。

契约：docs/guides/decision_control_interface.md（decision -> control）
      实际消息定义以 src/plugins/aic_msgs/msg/MaterialTask.msg 为准（uint8 + action 字符串）。

实现要点
--------
* 指令话题是 fire-and-forget：本节点维护 FIFO 队列，逐条执行，不抢决策的节奏。
* 运动用「梯形速度剖面 + 每 20 ms 流式下发 setpoint」的方式驱动 Gazebo 的
  JointPositionController（/aic/joint/{x,y,z}/cmd_pos，std_msgs/Float64）。
  这样在 x/y/z 之间是协调运动（同起同停），视觉上像真机，且时序可预测。
* 到位判定用「剖面走完 + 关节反馈收敛」，不再靠固定 settle 时间。
  早期版本按「剖面时间走完 + 0.15 s」就报到位，结果机台实际还在追：
  实测 p_gain=20 时，剖面 1.25 s 结束，实际位置到 4.8 s 才追上、过冲 17.7 mm、
  8.6 s 才停稳。于是「到位」那一刻相机看到的画面偏了整整一格，视觉必然失配。
  现在改为等 /joint_state 里 x/y/z 的 position 进 pos_tol、velocity 进 vel_tol，
  连续 confirm_ticks 拍都满足才报到位（超时按到位继续，只告警，不卡死流程）。
* 到位后发 /sim/gantry_arrived(UInt8=slot_id)，供 sim_vision_node 触发「拍照识别」；
  同时发 /vision/trigger(Empty)，供真视觉节点 aic_vision_trad 触发拍照
  （该话题是 vision 接口约定里的「采集请求」，见 docs/guides/decision_vision_interface.md）。
  HANDLE 做完发 /sim/handle_done(UInt8=slot_id)。

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
"""
import collections
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, UInt8

from aic_msgs.msg import MaterialTask
from sim_layout import SimLayout, declare_number

# 关节软限位，与 models/aic_gantry/model.sdf 的 <limit> 保持一致
LIMITS = {
    "x": (-0.45, 0.45),
    "y": (-0.24, 0.24),
    "z": (-0.42, 0.0),
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
        self.pick_z = float(self.layout.gantry.get("pick_z", -0.02))
        # 到位判据：位置阈值来自布局文件（与真机调参同一处），速度阈值默认 10 mm/s
        self.pos_tol = declare_number(self, "pos_tol", float(self.layout.gantry.get("pos_tol", 0.003)))
        self.vel_tol = declare_number(self, "vel_tol", float(self.layout.gantry.get("vel_tol", 0.010)))
        self.confirm_ticks = int(declare_number(self, "confirm_ticks", 4))
        self.move_timeout_s = declare_number(
            self, "move_timeout_s", float(self.layout.gantry.get("move_timeout_s", 8.0)))
        rate = declare_number(self, "control_rate_hz", 50.0)
        self.dt = 1.0 / rate

        self.pub = {a: self.create_publisher(Float64, "/aic/joint/{}/cmd_pos".format(a), 10)
                    for a in AXES}
        self.arrived_pub = self.create_publisher(UInt8, arrived_topic, 10)
        self.handled_pub = self.create_publisher(UInt8, handled_topic, 10)
        # 到位即发采集请求：真视觉节点（aic_vision_trad）订阅这条，与决策侧接口约定一致
        # （/vision/trigger，见 docs/guides/decision_vision_interface.md 第 1 节）。
        trigger_topic = self.declare_parameter("trigger_topic", "/vision/trigger").value
        self.trigger_pub = self.create_publisher(Empty, trigger_topic, 10)
        self.create_subscription(MaterialTask, task_topic, self._on_task, 10)

        # 就绪握手：decision 侧 control_wait_ready=true 时等这条 true 才开跑。
        # 必须周期重发——决策端是 volatile 订阅，只发一次会被后启动的节点错过。
        ready_topic = self.declare_parameter("ready_topic", "/control/ready").value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, ready_topic, qos)
        self._publish_ready()
        self.create_timer(1.0, self._publish_ready)

        # 理论位置（我们自己的 setpoint 账本）
        self.pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        # 实际位置/速度（来自 Gazebo 关节反馈；None = 还没收到，退回定时等待）
        self.actual = {a: None for a in AXES}
        self.vel = {a: 0.0 for a in AXES}
        self.queue = collections.deque()
        self.steps = collections.deque()   # 当前任务剩余步骤
        self.active = None                 # 正在执行的 move/dwell
        self.busy = False

        # 关节反馈订阅。桥在 sim.launch.py 里（ignition.msgs.Model -> sensor_msgs/JointState）。
        # QoS 用 BEST_EFFORT：gz 侧按 sensor data 发，reliable 订阅者会「静默收不到」。
        js_topic = self.declare_parameter(
            "joint_state_topic", "/world/aic_warehouse/model/aic_gantry/joint_state").value
        js_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, js_topic, self._on_joint_state, js_qos)

        self.create_timer(self.dt, self._tick)
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

    def _publish_ready(self):
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)

    def _on_task(self, msg):
        action = (msg.action or "").strip().upper()
        slot = int(msg.slot_id)
        kind = int(msg.material_type)
        self.get_logger().info(
            "[sim/control] 收到任务 %s slot=%d row=%d col=%d type=%d visit=%d"
            % (action, slot, msg.row, msg.col, kind, msg.visit_index))

        if action == "GOTO":
            self.queue.append(self._goto_steps(slot))
        elif action == "HANDLE":
            self.queue.append(self._handle_steps(slot))
        elif action == "HOME":
            self.queue.append(self._home_steps())
        else:
            self.get_logger().warn("[sim/control] 未知 action='%s'，忽略" % msg.action)

    def _goto_steps(self, slot):
        x, y = self.layout.slot_xy(slot)
        steps = []
        if self.pos["z"] < self.travel_z - 1e-4:
            steps.append(_Step("move", targets={"z": self.travel_z}, speed=self.z_speed))
        steps.append(_Step("move", targets={"x": x, "y": y}, speed=self.xy_speed))
        steps.append(_Step("settle", seconds=self.settle_s))
        steps.append(_Step("event", name="arrived", slot=slot))
        return steps

    def _handle_steps(self, slot):
        return [
            _Step("move", targets={"z": self.pick_z}, speed=self.z_speed),
            _Step("dwell", seconds=self.dwell_s),
            _Step("move", targets={"z": self.travel_z}, speed=self.z_speed),
            _Step("event", name="handled", slot=slot),
        ]

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
            if step.kind == "event":
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
        if step.hold_s >= self.move_timeout_s:
            self.get_logger().warn(
                "[sim/control] 到位超时 %.1f s（%s），按到位继续"
                % (step.hold_s, detail))
            self.active = None

    def _converged(self, step):
        """判断 step.targets 里的轴是否已真正停稳。返回 (bool, 说明)。"""
        if any(self.actual[a] is None for a in AXES):
            return step.hold_s >= self.settle_s, "无关节反馈，按 %.2f s 定时等待" % self.settle_s
        worst_pos = 0.0
        worst_vel = 0.0
        for a in step.targets:
            worst_pos = max(worst_pos, abs(self.actual[a] - step.targets[a]))
            worst_vel = max(worst_vel, abs(self.vel[a]))
        return (worst_pos <= self.pos_tol and worst_vel <= self.vel_tol,
                "最大误差 %.1f mm，最大速度 %.1f mm/s"
                % (worst_pos * 1000.0, worst_vel * 1000.0))

    def _publish(self, axis):
        msg = Float64()
        msg.data = float(self.pos[axis])
        self.pub[axis].publish(msg)

    def _emit(self, step):
        if step.name == "arrived":
            msg = UInt8()
            msg.data = int(step.slot)
            self.arrived_pub.publish(msg)
            # 采集请求：真视觉链路靠这条触发拍照（ground-truth 桩不订阅它，无副作用）
            self.trigger_pub.publish(Empty())
            self.get_logger().info("[sim/control] slot %d 到位，已通知 vision" % step.slot)
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
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
