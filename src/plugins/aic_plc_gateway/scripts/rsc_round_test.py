#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""RSC 后端整轮联调：一个进程里同时扮演 PLC、控制器、视觉，走完整握手。

为什么要这样一个脚本
--------------------
Modbus 后端可以用 plc_master_demo.py 跨进程驱动（主站连 5020）；
RSC 后端没有 vPLC 时只有「内存假 PLC」，而假 PLC 是进程内对象，
外部进程看不到。所以这里单进程跑：

    本脚本（PLC 侧）  写 TASK_CMD/POS_SET  ──►  网关节点  ──► 发布 GOTO/HANDLE
    本脚本（控制器）  直接调 _on_arrived / _on_handled
    本脚本（视觉）    直接调 _on_vision

覆盖的是：标签映射、状态机迁移、写入类型（含负坐标）、心跳。
覆盖不了的是：现场变量名对不对（那要真 vPLC，见 plcnext_rsc_setup.md §5）。

用法
----
    ros2 run aic_plc_gateway rsc_round_test.py     # 或直接 python3 跑
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, UInt8

from plc_gateway import (AX_INPOS, CAM_DONE, PlcGateway, TK_DONE, TK_GOTO,
                         TK_HANDLE)

FAILS = []


def check(cond, msg):
    print(("  OK   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


def plc_write(node, tag, value):
    """扮演 PLC：直接写进内存假 PLC 的变量表。"""
    link = node.io.link
    spec = link.map.tags[tag]
    link._svc.store[spec.path] = link.sdk.RscVariant(value, link._type_of(spec.iec))


def plc_read(node, tag):
    """扮演 PLC：从假 PLC 读回一个标签（含网关写的那些）。"""
    vals = node.io.link.read_all(include_writes=True)
    return vals.get(tag)


def main():
    rclpy.init(args=["--ros-args",
                     "-p", "backend:=rsc",
                     "-p", "rsc_dry_run:=true",
                     "-p", "allow_control:=true",
                     "-p", "poll_hz:=50.0"])

    node = PlcGateway()
    sent = []

    def _loopback(m):
        """截住下发的任务，同时模拟 ROS 回环。

        真实节点订阅了自己发布的 /control/material_task（_on_task），
        靠这条回环把 task_state 置位；本脚本不 spin executor，所以手动喂一次。
        """
        sent.append(m)
        node._on_task(m)

    node.task_pub.publish = _loopback

    print("后端：%s" % node.io.describe())

    # 1) 空转一拍：应当把 21 个输出标签写下去
    node._tick()
    check(plc_read(node, "TASK_STATE") == 0, "初始 TASK_STATE=0")
    check(plc_read(node, "DI_INPOS") is False, "初始 DI_INPOS=false")

    # 2) PLC 下发 GOTO（POS_SET 与 TASK_CMD 是两个变量，分开写）
    plc_write(node, "POS_SET", 3)
    plc_write(node, "TASK_CMD", 1)
    node._tick()
    check(node.task_state == TK_GOTO, "TASK_CMD=1 -> 状态机进 GOTO")
    check(len(sent) == 1 and sent[-1].action == "GOTO" and sent[-1].slot_id == 3,
          "下发 GOTO slot=3（实际 %r）"
          % ((None if not sent else (sent[-1].action, sent[-1].slot_id)),))

    # 3) 控制器到位 + 视觉出结果
    node._on_arrived(UInt8(data=3))
    node._tick()
    check(node.axis_state == AX_INPOS, "到位后 AXIS_STATE=INPOS")
    check(plc_read(node, "DI_INPOS") is True, "DI_INPOS 已置位（PLC 可据此拍照）")

    node._on_vision(String(data="2"))
    node._tick()
    check(node.cam_state == CAM_DONE, "视觉结果 -> CAM_STATE=DONE")
    check(plc_read(node, "CAM_DATA") == 2, "CAM_DATA=2 写回 PLC")

    # 4) PLC 复位执行位（RSC 下 TASK_CMD 归 PLC 所有，网关写不动它）
    plc_write(node, "TASK_CMD", 0)
    node._tick()

    # 5) PLC 下发 HANDLE，必须带上识别类型
    plc_write(node, "TASK_CMD", 2)
    node._tick()
    check(node.task_state == TK_HANDLE, "TASK_CMD=2 -> 状态机进 HANDLE")
    check(sent[-1].action == "HANDLE" and sent[-1].material_type == 2,
          "HANDLE 携带 material_type=2（实际 %r）"
          % ((None if not sent else (sent[-1].action, sent[-1].material_type)),))

    # 6) 抓放完成
    node._on_handled(UInt8(data=3))
    node._tick()
    check(node.task_state == TK_DONE, "handle_done -> TASK_STATE=DONE")
    check(plc_read(node, "SHELF_BITS") == (1 << 2), "SHELF_BITS 第 3 位置位")
    check(plc_read(node, "PLACED_CNT") == 1, "PLACED_CNT=1")

    # 7) 负坐标：RSC 必须是真负数，不能像 Modbus 那样折成 uint16
    node.actual["x"] = -0.010
    node._tick()
    check(plc_read(node, "X_ACT") == -10, "X_ACT=-10 mm（有符号，不是 65526）")

    # 8) 心跳
    hb0 = plc_read(node, "HEARTBEAT")
    node._hb_t -= 1.1            # 把上次心跳时间往前拨，逼它 +1
    node._tick()
    check(plc_read(node, "HEARTBEAT") == hb0 + 1, "HEARTBEAT 递增")

    node.io.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    print()
    if FAILS:
        print("失败 %d 项：" % len(FAILS))
        for f in FAILS:
            print("  - " + f)
        return 1
    print("全部通过：RSC 后端整轮握手 OK")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ExternalShutdownException:
        sys.exit(0)
