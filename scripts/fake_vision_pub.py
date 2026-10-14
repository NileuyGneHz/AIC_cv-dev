#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""假 vision：收到决策下发的 GOTO 任务后，回一条物料类型字符串。

真 vision 模块未实现之前的**开发期桩**（不进入比赛提交物）。

用法：
  ros2 run 场景下也可直接：
  /usr/bin/python3 scripts/fake_vision_pub.py --mode sequence
  /usr/bin/python3 scripts/fake_vision_pub.py --mode sequence --sequence 1,2,3,4 --invalid-every 5
  /usr/bin/python3 scripts/fake_vision_pub.py --mode silent          # 验证决策的超时路径

模式：
  sequence（默认）：按 --sequence 顺序取值（用完循环）
  random          ：1..4 随机
  invalid         ：一直发 --invalid-value（默认 "9"）
  silent          ：什么都不发

说明：本脚本用 /usr/bin/python3（系统 3.10）——本机 PATH 里另有别的 python3，那份没有 rclpy。
"""
import argparse
import random
import sys
import threading

import rclpy
from aic_msgs.msg import MaterialTask
from rclpy.node import Node
from std_msgs.msg import String


class FakeVision(Node):

    def __init__(self, args):
        super().__init__('fake_vision')
        self.args = args
        self.sequence = [s.strip() for s in args.sequence.split(',') if s.strip()]
        self.pub = self.create_publisher(String, args.vision_topic, 10)
        self.create_subscription(MaterialTask, args.task_topic, self.on_task, 10)

        self.received_goto = 0
        self.published = 0
        self.pending = []          # 延迟发布队列
        self.lock = threading.Lock()
        self.create_timer(0.02, self.drain)

        self.get_logger().info(
            "假 vision 就绪：监听 %s 的 GOTO，往 %s 发（mode=%s, sequence=%s, invalid_every=%d, delay=%.0fms）"
            % (args.task_topic, args.vision_topic, args.mode, self.sequence or '-',
               args.invalid_every, args.delay_ms))

    # ---------------------------------------------------------------- 回调
    def on_task(self, msg: MaterialTask):
        if msg.action != MaterialTask.ACTION_GOTO:
            return
        self.received_goto += 1

        if self.args.mode == 'silent':
            self.get_logger().info(
                "GOTO#%d slot%02d -> 静默模式，不回消息" % (self.received_goto, msg.slot_id))
            return

        value = self.next_value()
        self.get_logger().info(
            "GOTO#%d slot%02d (visit%02d) -> 发布 '%s'"
            % (self.received_goto, msg.slot_id, msg.visit_index, value))

        if self.args.delay_ms > 0:
            with self.lock:
                self.pending.append(value)
        else:
            self.publish(value)

    def next_value(self) -> str:
        idx = self.published + 1
        if self.args.invalid_every > 0 and idx % self.args.invalid_every == 0:
            return self.args.invalid_value
        if self.args.mode == 'invalid':
            return self.args.invalid_value
        if self.args.mode == 'random':
            return str(random.randint(1, 4))
        return self.sequence[(idx - 1) % len(self.sequence)]

    def publish(self, value: str):
        msg = String()
        msg.data = value
        self.pub.publish(msg)
        self.published += 1

    def drain(self):
        with self.lock:
            pending, self.pending = self.pending, []
        for value in pending:
            self.publish(value)


def main():
    parser = argparse.ArgumentParser(description='假 vision 桩：按 GOTO 应答物料类型')
    parser.add_argument('--task-topic', default='/control/material_task')
    parser.add_argument('--vision-topic', default='/vision/material_type')
    parser.add_argument('--mode', default='sequence',
                        choices=['sequence', 'random', 'invalid', 'silent'])
    parser.add_argument('--sequence', default='1,2,3,4')
    parser.add_argument('--invalid-value', default='9')
    parser.add_argument('--invalid-every', type=int, default=0,
                        help='每 N 条里发一条非法值（0 = 从不）')
    parser.add_argument('--delay-ms', type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = FakeVision(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            "退出：收到 GOTO %d 次，发布 %d 条" % (node.received_goto, node.published))
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
