#!/bin/bash
# 端到端单轮：PLC 接管只跑 1 块料（改完代码先跑这个，约 30 s）。
# 判定标准：主站打印「成功 1/1」，抓放完成 ≈ 12–13 s（不是 0.3 s）。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u          # 必须放在 source 之后：ROS 的 setup.bash 会引用未定义变量
export ROS_HOME=$PWD/logs/.ros ROS_LOG_DIR=$PWD/logs/.ros/log ROS_LOCALHOST_ONLY=1
# 坑：ros2 run 走 #!/usr/bin/env python3，会命中 WorkBuddy 托管的 3.13（没有 cv2）。
# 视觉节点必须用系统 python3.10，所以把 /usr/bin 放最前。
export PATH=/usr/bin:$PATH

PORT="${PORT:-5021}"
# 坑：pkill -f gzserver 在 Fortress 上匹配 0 个进程（进程名是 `ign gazebo`），
#     上一轮实例会存活并和新实例抢关节指令。用统一清理库。
# shellcheck source=lib/sim_cleanup.sh
source "$ROOT/scripts/lib/sim_cleanup.sh"
sim_kill_stale
ros2 launch aic_dashboard dashboard.launch.py vision_backend:=trad start_decision:=false \
  > logs/plc_one_sim.log 2>&1 &
SIM=$!
cleanup() {
  kill -INT "${GW:-0}" 2>/dev/null || true
  kill -INT "${SIM:-0}" 2>/dev/null || true
  sleep 3
  # 收尾再清一遍：ros2 launch 的子进程常常比 launch 本身活得久（reparent 到 1）
  sim_kill_stale
}
trap cleanup EXIT
sleep 14

ros2 launch aic_plc_gateway plc_gateway.launch.py port:="$PORT" allow_control:=true \
  > logs/plc_one_gw.log 2>&1 &
GW=$!
sleep 4

/usr/bin/python3 "$ROOT/src/plugins/aic_plc_gateway/scripts/plc_master_demo.py" \
  --mode control --port "$PORT" --materials 1 | tee logs/plc_one_master.txt
