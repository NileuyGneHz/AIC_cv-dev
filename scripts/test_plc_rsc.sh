#!/bin/bash
# RSC 后端整轮联调：干跑假 PLC，不需要 vPLC，也不需要仿真（约 3 s）。
# 判定标准：末尾打印「全部通过：RSC 后端整轮握手 OK」。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u          # 必须放在 source 之后：ROS 的 setup.bash 会引用未定义变量
export ROS_HOME=$PWD/logs/.ros ROS_LOG_DIR=$PWD/logs/.ros/log ROS_LOCALHOST_ONLY=1
# 坑：ros2 run 走 #!/usr/bin/env python3，会命中 WorkBuddy 托管的 3.13（没有 cv2）。
# 视觉节点必须用系统 python3.10，所以把 /usr/bin 放最前。
export PATH=/usr/bin:$PATH

ros2 run aic_plc_gateway rsc_round_test.py
