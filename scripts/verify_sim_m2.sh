#!/usr/bin/env bash
# M2 搬运闭环自验：真视觉 + 决策 + 抓取码放跑一整轮，然后断言三件事。
#
#   1) 决策结果 12 行全 OK，且每个位点的类型与 sim_layout.yaml 的 materials.types 一致
#   2) 日志里「夹持」「释放」各 12 次（决策退出不等于控制队列排空，见指南坑 19）
#   3) 12 块铁片都落在对应类型的码放格里（x/y 容差 4 mm，z 在台面高度）
#
# 用法：
#   ./scripts/verify_sim_m2.sh                 # 真视觉（默认），约 100 s
#   ./scripts/verify_sim_m2.sh --vision gt     # 真值桩（排除视觉因素，定位控制/物理问题）
#   ./scripts/verify_sim_m2.sh --timeout 300   # 机器慢时放宽等待上限
#
# 产物：logs/verify_m2_<stamp>.csv|log、logs/verify_m2_pose_<stamp>.txt
set -uo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

if [[ ! -f install/setup.bash ]]; then
  echo "[verify] 还没有 install/，先执行 ./scripts/build.sh" >&2
  exit 2
fi

VISION="trad"
LIMIT=240
while [[ $# -gt 0 ]]; do
  case "$1" in
    --vision) VISION="$2"; shift 2 ;;
    --timeout) LIMIT="$2"; shift 2 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "[verify] 未知参数：$1" >&2; exit 2 ;;
  esac
done

# 坑：ROS 的 setup.bash 会引用未定义变量，与 set -u 冲突，source 期间先关掉
set +u
# shellcheck disable=SC1090
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source install/setup.bash
set -u
export PATH="/usr/bin:${PATH}"
export ROS_HOME="${ROS_HOME:-${WS_DIR}/logs/.ros}"
export ROS_LOG_DIR="${ROS_HOME}/log"
mkdir -p "${ROS_LOG_DIR}" logs
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

STAMP="$(date +%Y%m%d_%H%M%S)"
CSV="logs/verify_m2_${STAMP}.csv"
LOG="logs/verify_m2_${STAMP}.log"
POSE="logs/verify_m2_pose_${STAMP}.txt"

# 布局里的位点数，作为「等几个 detach」的期望值（不写死 12）
N_SLOTS=$(/usr/bin/python3 -c "
import yaml
print(len(yaml.safe_load(open('src/simulation/aic_gazebo_sim/config/sim_layout.yaml'))['materials']['types']))
")

echo "[verify] vision=${VISION} 位点数=${N_SLOTS} 结果 CSV=${CSV}"

ros2 launch aic_gazebo_sim sim_decision.launch.py \
  once:=false \
  vision_backend:="${VISION}" \
  results_csv:="${CSV}" > "${LOG}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT "${LAUNCH_PID}" 2>/dev/null
  sleep 3
  pkill -f gzserver 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null
}
trap cleanup EXIT

# 坑：不要用固定 sleep 等结束 —— 最后一块铁片还在飞行中就被 kill 的话，
#     位姿会停在半空、attach 计数也少一次，看着像「搬运失败」。
#     正确做法是轮询日志里的 detach 计数，等它等于位点数再抓位姿。
echo "[verify] 等待 ${N_SLOTS} 次释放（上限 ${LIMIT}s）..."
for ((t = 0; t < LIMIT; t++)); do
  sleep 1
  n=$(grep -c '释放（DetachableJoint detach）' "${LOG}" 2>/dev/null || true)
  if [[ "${n:-0}" -ge "${N_SLOTS}" ]]; then
    echo "[verify] 释放已达 ${n} 次（t=${t}s），等 3 s 让铁片落稳"
    break
  fi
done

sleep 3
timeout 10 ign topic -e -t /world/aic_warehouse/pose/info -n 1 > "${POSE}" 2>&1 || true

echo "[verify] 断言 -----------------------------------------------------------------"
/usr/bin/python3 scripts/check_sim_m2.py \
  --results "${CSV}" --log "${LOG}" --pose "${POSE}"
RC=$?

if [[ ${RC} -ne 0 ]]; then
  echo "[verify] 失败现场：${LOG} / ${POSE}"
else
  echo "[verify] 通过。产物：${CSV}"
fi
exit ${RC}
