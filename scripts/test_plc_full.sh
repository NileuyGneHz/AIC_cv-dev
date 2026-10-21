#!/bin/bash
# 端到端整轮：PLC 接管跑完 12 块料（约 160 s）。
# 判定标准：主站打印「成功 12/12」+ 货架 111111111111 + 放满标志=1。
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
  > logs/plc_full_sim.log 2>&1 &
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
  > logs/plc_full_gw.log 2>&1 &
GW=$!
sleep 4

/usr/bin/python3 "$ROOT/src/plugins/aic_plc_gateway/scripts/plc_master_demo.py" \
  --mode control --port "$PORT" --materials 12 > logs/plc_full_master.txt 2>&1
RC=$?
cat logs/plc_full_master.txt
echo "MASTER_RC=$RC"

# ---- 物理落位核对 ----
# 为什么必须做：网关的 SHELF_BITS 是「收到 detach 事件」累出来的，不是真实坐标。
# 控制器在「到位超时」时会「按到位继续」，于是一次 RTF 抖动就可能把料放在格口外，
# 而 PLC 侧仍显示 12/12。这里用与决策路径同一套断言核对物理位置。
if [ "$RC" -eq 0 ]; then
  STAMP=$(date +%Y%m%d_%H%M%S)
  POSE="logs/plc_full_pose_${STAMP}.txt"
  CSV="logs/plc_full_results_${STAMP}.csv"
  sleep 4                                  # 等最后一块落稳、吸盘抬起

  for attempt in 1 2 3; do
    timeout 20 ign topic -e -t /world/aic_warehouse/pose/info -n 1 > "$POSE" 2>&1 || true
    if grep -q 'material_' "$POSE" 2>/dev/null; then break; fi
    echo "[plc] 位姿转储为空，重试 ${attempt}/3"
    sleep 2
  done

  /usr/bin/python3 "$ROOT/scripts/plc_master_to_csv.py" logs/plc_full_master.txt > "$CSV"
  echo "[plc] 物理落位核对 ---------------------------------------------------"
  /usr/bin/python3 "$ROOT/scripts/check_sim_m2.py" \
    --results "$CSV" --log logs/plc_full_sim.log --pose "$POSE" --label 主站
  CHECK_RC=$?
  if [ "$CHECK_RC" -ne 0 ]; then
    echo "[plc] 落位失败现场：$POSE / logs/plc_full_sim.log"
  fi
else
  CHECK_RC=$RC
fi
exit "$CHECK_RC"
