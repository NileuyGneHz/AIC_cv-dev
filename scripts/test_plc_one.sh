#!/bin/bash
# 端到端单轮：PLC 接管只跑 1 块料（改完代码先跑这个，约 30 s）。
# 判定标准：主站打印「成功 1/1」，抓放完成 ≈ 12–13 s（不是 0.3 s）。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u          # 必须放在 source 之后：ROS 的 setup.bash 会引用未定义变量
export ROS_HOME=$PWD/logs/.ros ROS_LOG_DIR=$PWD/logs/.ros/log ROS_LOCALHOST_ONLY=1

PORT="${PORT:-5021}"
pkill -f gzserver 2>/dev/null; sleep 1
ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad start_decision:=false \
  > logs/plc_one_sim.log 2>&1 &
SIM=$!
trap 'kill -INT $GW 2>/dev/null; kill -INT $SIM 2>/dev/null' EXIT
sleep 14

ros2 launch aic_plc_gateway plc_gateway.launch.py port:="$PORT" allow_control:=true \
  > logs/plc_one_gw.log 2>&1 &
GW=$!
sleep 4

/usr/bin/python3 "$ROOT/src/plugins/aic_plc_gateway/scripts/plc_master_demo.py" \
  --mode control --port "$PORT" --materials 1 | tee logs/plc_one_master.txt
