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
#   ./scripts/verify_sim_m2.sh --domain auto   # 隔离实例：同机还有别的仿真在跑时用（推荐）
#   ./scripts/verify_sim_m2.sh --domain 7      # 指定固定号（只在确认没人用时）
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
DEBUG_DIR=""
DOMAIN=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --vision) VISION="$2"; shift 2 ;;
    --timeout) LIMIT="$2"; shift 2 ;;
    --debug-dir) DEBUG_DIR="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
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

# 并行隔离：给每个实例独立的 ROS domain 与 Gazebo partition（坑 23）。
# 不起作用的话，两套仿真会共用 /world/aic_warehouse/* 话题，真值桩替对方秒回类型。
# shellcheck source=lib/sim_domain.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/sim_domain.sh"
if [[ "${DOMAIN}" == "auto" ]]; then
  # 手挑固定号会撞：实测两个会话都选 7，同一分区里 53 条话题（两套 × 26 条）。
  # 自动分配会扫进程表 + 探测空分区，并把占用的号登记到 logs/.sim_domains。
  DOMAIN="$(sim_alloc_domain)" || { echo "[verify] 找不到空闲 domain（1-${SIM_DOMAIN_MAX} 都被占）" >&2; exit 2; }
  echo "[verify] 自动分配 domain=${DOMAIN}"
fi
if [[ -n "${DOMAIN}" ]]; then
  sim_apply_domain "${DOMAIN}"
  echo "[verify] 隔离：ROS_DOMAIN_ID=${ROS_DOMAIN_ID} IGN_PARTITION=${IGN_PARTITION}"
fi

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

# 起跑前检查：两个仿真共用一个 ROS domain 会互相污染（真值桩秒回类型、夹放对不上），
# 现象极像「视觉退化」。这里**不替用户杀进程**（同一台机器上可能还有别的窗口在跑），
# 只报告并退出，让人自己决定。给了 --domain 就跳过 —— 那种情况已经隔离，可以并行。
if [[ -z "${DOMAIN}" ]] \
   && { pgrep -f 'aic_gazebo_sim/lib/aic_gazebo_sim' >/dev/null 2>&1 \
        || pgrep -f 'aic_vision_trad/lib/aic_vision_trad' >/dev/null 2>&1 \
        || pgrep -f 'aic_behavior_tree/lib/aic_behavior_tree' >/dev/null 2>&1; }; then
  echo "[verify] 已有仿真节点在跑（另一个窗口 / 上一轮没退干净）。" >&2
  echo "[verify] 两个仿真共用 ROS domain 会让结果整片错位；要么先停掉它，" >&2
  echo "[verify] 要么用 ./scripts/verify_sim_m2.sh --domain auto 跑隔离实例。" >&2
  exit 3
fi

EXTRA_ARGS=()
if [[ -n "${DEBUG_DIR}" ]]; then
  mkdir -p "${DEBUG_DIR}"
  EXTRA_ARGS+=("vision_debug_dir:=${DEBUG_DIR}")
fi

ros2 launch aic_gazebo_sim sim_decision.launch.py \
  once:=false \
  vision_backend:="${VISION}" \
  results_csv:="${CSV}" "${EXTRA_ARGS[@]}" > "${LOG}" 2>&1 &
LAUNCH_PID=$!

# 只清自己这一棵进程树，不用全局 pkill —— 同机可能有别人的仿真在跑。
# 坑：只 kill launch 本身不够，子节点（gantry_controller / vision_node / bt_runner /
#     gzserver / 桥）会变成孤儿继续发消息，下一轮就被它们污染。
kill_tree() {
  local pid="$1" child
  for child in $(pgrep -P "${pid}" 2>/dev/null); do
    kill_tree "${child}"
  done
  kill -TERM "${pid}" 2>/dev/null
}
cleanup() {
  kill -INT "${LAUNCH_PID}" 2>/dev/null
  sleep 2
  pgrep -P "${LAUNCH_PID}" >/dev/null 2>&1 && kill_tree "${LAUNCH_PID}"
  sleep 1
  kill -KILL "${LAUNCH_PID}" 2>/dev/null
  [[ -n "${DOMAIN}" ]] && sim_release_domain
}
trap cleanup EXIT

# 坑：不要用固定 sleep 等结束 —— 最后一块铁片还在飞行中就被 kill 的话，
#     位姿会停在半空、attach 计数也少一次，看着像「搬运失败」。
#     正确做法是轮询日志里的 detach 计数，等它等于位点数再抓位姿。
echo "[verify] 等待 ${N_SLOTS} 次释放（上限 ${LIMIT}s）..."
for ((t = 0; t < LIMIT; t++)); do
  sleep 1
  n=$(grep -c '释放（吸盘断气 / DetachableJoint detach）' "${LOG}" 2>/dev/null || true)
  if [[ "${n:-0}" -ge "${N_SLOTS}" ]]; then
    echo "[verify] 释放已达 ${n} 次（t=${t}s），等 3 s 让铁片落稳"
    break
  fi
done

sleep 3
# 坑：ign topic -e -n 1 偶尔一条也收不到（传输发现慢 / 上一条订阅还没清干净），
#     单次失败会让整轮结论作废（表现为「pose/info 里只有 0 块铁片」）。
#     抓 3 次，谁先拿到非空就用谁。
for attempt in 1 2 3; do
  timeout 20 ign topic -e -t /world/aic_warehouse/pose/info -n 1 > "${POSE}" 2>&1 || true
  if grep -q 'material_' "${POSE}" 2>/dev/null; then
    break
  fi
  echo "[verify] 位姿转储为空，重试 ${attempt}/3"
  sleep 2
done

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
