#!/bin/bash
# 端到端整轮：PLC 接管跑完 12 块料（约 160 s）。
# 判定标准：主站打印「成功 12/12」+ 货架 111111111111 + 放满标志=1。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u          # 必须放在 source 之后：ROS 的 setup.bash 会引用未定义变量
export ROS_HOME=$PWD/logs/.ros ROS_LOG_DIR=$PWD/logs/.ros/log ROS_LOCALHOST_ONLY=1

PORT="${PORT:-5021}"
pkill -f gzserver 2>/dev/null; sleep 1
ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad start_decision:=false \
  > logs/plc_full_sim.log 2>&1 &
SIM=$!
trap 'kill -INT $GW 2>/dev/null; kill -INT $SIM 2>/dev/null' EXIT
sleep 14

ros2 launch aic_plc_gateway plc_gateway.launch.py port:="$PORT" allow_control:=true \
  > logs/plc_full_gw.log 2>&1 &
GW=$!
sleep 4

/usr/bin/python3 "$ROOT/src/plugins/aic_plc_gateway/scripts/plc_master_demo.py" \
  --mode control --port "$PORT" --materials 12 > logs/plc_full_master.txt 2>&1
RC=$?
cat logs/plc_full_master.txt
echo "MASTER_RC=$RC"
