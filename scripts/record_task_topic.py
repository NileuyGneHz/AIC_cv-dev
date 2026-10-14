#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""记录决策下发的 control 任务话题，证明「任务真的发到了线上」（而不是只看节点自报日志）。

用法：
  /usr/bin/python3 scripts/record_task_topic.py --out logs/tasks.csv --expect-handle 12
  /usr/bin/python3 scripts/record_task_topic.py --duration 20

退出条件：收到 --expect-handle 条 HANDLE（>0 时生效）或到达 --duration 秒。
退出时打印汇总（按动作计数、GOTO 位点顺序、HANDLE 类型分布）。

用 /usr/bin/python3（系统 3.10）：本机 PATH 里另有别的 python3，那份没有 rclpy。
"""
import argparse
import csv
import sys
import time

import rclpy
from aic_msgs.msg import MaterialTask
from rclpy.node import Node


class TaskRecorder(Node):

    def __init__(self, args):
        super().__init__('task_recorder')
        self.args = args
        self.rows = []
        self.gotos = []
        self.handles = []
        self.deadline = time.monotonic() + args.duration
        self.create_subscription(MaterialTask, args.topic, self.on_task, 10)
        self.create_timer(0.1, self.check_deadline)
        self.get_logger().info(
            "记录 '%s'：最多 %.1f s，或收到 %d 条 HANDLE 后退出 -> %s"
            % (args.topic, args.duration, args.expect_handle, args.out))

    def on_task(self, msg: MaterialTask):
        self.rows.append({
            'action': msg.action,
            'visit_index': msg.visit_index,
            'slot_id': msg.slot_id,
            'row': msg.row,
            'col': msg.col,
            'material_type': msg.material_type,
            'note': msg.note,
        })
        if msg.action == MaterialTask.ACTION_GOTO:
            self.gotos.append((msg.visit_index, msg.slot_id, msg.row, msg.col))
        elif msg.action == MaterialTask.ACTION_HANDLE:
            self.handles.append((msg.visit_index, msg.slot_id, msg.material_type))

        if self.args.expect_handle > 0 and len(self.handles) >= self.args.expect_handle:
            self.get_logger().info("已收满 %d 条 HANDLE，结束记录" % len(self.handles))
            rclpy.shutdown()

    def check_deadline(self):
        if time.monotonic() >= self.deadline:
            self.get_logger().info("到达记录时长上限，结束记录")
            rclpy.shutdown()

    def write_csv(self):
        with open(self.args.out, 'w', newline='', encoding='utf-8') as fh:
            writer = csv.DictWriter(fh, fieldnames=[
                'action', 'visit_index', 'slot_id', 'row', 'col', 'material_type', 'note'])
            writer.writeheader()
            writer.writerows(self.rows)

    def summary(self):
        types = {}
        for _, _, t in self.handles:
            types[t] = types.get(t, 0) + 1
        lines = [
            "===== control 任务话题记录 =====",
            "  总消息：%d（GOTO %d / HANDLE %d）" % (len(self.rows), len(self.gotos), len(self.handles)),
            "  GOTO 位点顺序：" + " ".join(
                "%02d:slot%02d(r%d,c%d)" % g for g in self.gotos),
            "  HANDLE 类型分布：" + " ".join(
                "type%d=%d" % (k, v) for k, v in sorted(types.items())),
            "  CSV：%s" % self.args.out,
        ]
        return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description='记录 aic_msgs/MaterialTask 话题')
    parser.add_argument('--topic', default='/control/material_task')
    parser.add_argument('--out', default='logs/tasks.csv')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--expect-handle', type=int, default=0)
    args = parser.parse_args()

    rclpy.init()
    node = TaskRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_csv()
        print(node.summary(), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
