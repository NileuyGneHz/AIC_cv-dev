#!/usr/bin/python3
"""视觉侧仿真桩（ground truth 版）：机台到位后按布局表回答缺陷类型。

契约：docs/guides/decision_vision_interface.md —— 每次 GOTO 后最多一条
      std_msgs/String，取值 "1".."4"；失败场景「不发」即可，不发特殊值。

为什么先做 ground truth 桩
--------------------------
真视觉（OpenCV / YOLO）还没写。这个桩让决策-控制-仿真三段先跑通闭环，
并且可以注入 drop_rate / invalid_rate 复现「识别失败 / 脏数据」路径
（对应决策侧 TIMEOUT 兜底与非法值丢弃计数）。
等 aic_vision_trad 就绪后，把本节点换掉即可，话题契约不变。

触发方式：订阅 /sim/gantry_arrived（std_msgs/UInt8 = slot_id），
延时 settle_s + delay_s 后发一条；不依赖相机图像，也不看 /vision/trigger。
"""
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8

from sim_layout import SimLayout, declare_number


class SimVisionNode(Node):
    def __init__(self):
        super().__init__("sim_vision_node")

        self.layout = SimLayout.load(self.declare_parameter("layout_yaml", "").value)
        arrived_topic = self.declare_parameter("arrived_topic", "/sim/gantry_arrived").value
        self.vision_topic = self.declare_parameter("vision_topic", "/vision/material_type").value
        self.settle_s = declare_number(self, "settle_s", 0.20)
        self.delay_s = declare_number(self, "delay_s", 0.0)
        self.drop_rate = declare_number(self, "drop_rate", 0.0)
        self.invalid_rate = declare_number(self, "invalid_rate", 0.0)
        seed = declare_number(self, "seed", 0, int)

        self.rng = random.Random(seed)
        self.pub = self.create_publisher(String, self.vision_topic, 10)
        self.create_subscription(UInt8, arrived_topic, self._on_arrived, 10)
        self._timers = []

        self.get_logger().info(
            "[sim/vision] ground-truth 桩就绪：%s -> %s（delay=%.2fs drop=%.2f invalid=%.2f）"
            % (arrived_topic, self.vision_topic,
               self.settle_s + self.delay_s, self.drop_rate, self.invalid_rate))
        self.get_logger().info("[sim/vision] %s" % self.layout.summary())

    def _on_arrived(self, msg):
        slot = int(msg.data)
        if slot <= 0:
            return
        if self.drop_rate > 0.0 and self.rng.random() < self.drop_rate:
            self.get_logger().warn("[sim/vision] slot %d 模拟识别失败（不发结果）" % slot)
            return

        delay = self.settle_s + self.delay_s
        timer = self.create_timer(delay, lambda: self._fire(timer, slot))
        self._timers.append(timer)

    def _fire(self, timer, slot):
        timer.cancel()
        self.destroy_timer(timer)
        if timer in self._timers:
            self._timers.remove(timer)

        mtype = self.layout.slot_type(slot)
        payload = str(mtype)
        if mtype not in (1, 2, 3, 4):
            self.get_logger().error("[sim/vision] slot %d 在布局表里没有类型，不发" % slot)
            return
        if self.invalid_rate > 0.0 and self.rng.random() < self.invalid_rate:
            payload = "9"  # 非法值：决策侧应丢弃并计数，本位点最终记 TIMEOUT

        msg = String()
        msg.data = payload
        self.pub.publish(msg)
        self.get_logger().info("[sim/vision] slot %d -> '%s'" % (slot, payload))


def main():
    rclpy.init()
    node = SimVisionNode()
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
