#!/usr/bin/env bash
# 一键起仿真并跑一轮巡视（仿真 + 龙门控制 + ground-truth 视觉桩 + 决策行为树）。
#
# 用法：
#   ./scripts/run_sim.sh                 # 无头（默认），跑完一轮自动退出
#   ./scripts/run_sim.sh --gui           # 带 Gazebo GUI 看画面
#   ./scripts/run_sim.sh --loop          # 常驻多轮（Ctrl-C 结束）
#   ./scripts/run_sim.sh --drop-rate 0.3 # 注入 30% 识别失败（验证 TIMEOUT 兜底）
#   ./scripts/run_sim.sh --vision trad   # 换真视觉（aic_vision_trad 读相机图，不用真值桩）
#   ./scripts/run_sim.sh --debug-dir logs/vision_debug   # 真视觉存标注图
#   ./scripts/run_sim.sh --results logs/my.csv
#   ./scripts/run_sim.sh --domain auto    # 隔离实例（同机跑别的仿真时用，自动挑空闲号）
#
# 产物：logs/sim_launch_<stamp>.log、决策结果 CSV（默认 logs/sim_results_<stamp>.csv）
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

if [[ ! -f install/setup.bash ]]; then
  echo "[sim] 还没有 install/，先执行 ./scripts/build.sh" >&2
  exit 2
fi

# 坑：ROS 的 setup.bash 会引用未定义变量，与 set -u 冲突，source 期间先关掉
set +u
# shellcheck disable=SC1090
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source install/setup.bash
set -u
export PATH="/usr/bin:${PATH}"

# 坑：rcl 初始化要写 ~/.ros/log；HOME 不可写时节点直接起不来
export ROS_HOME="${ROS_HOME:-${WS_DIR}/logs/.ros}"
export ROS_LOG_DIR="${ROS_HOME}/log"
mkdir -p "${ROS_LOG_DIR}"
# 单机自验，避免多网卡/多机发现干扰
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

GUI="false"
ONCE="true"
AUTORESTART="false"
DROP="0.0"
INVALID="0.0"
VISION="gt"
DEBUG_DIR=""
DOMAIN=""
STAMP="$(date +%Y%m%d_%H%M%S)"
RESULTS="logs/sim_results_${STAMP}.csv"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui) GUI="true"; shift ;;
    --loop) ONCE="false"; AUTORESTART="true"; shift ;;
    --drop-rate) DROP="$2"; shift 2 ;;
    --invalid-rate) INVALID="$2"; shift 2 ;;
    --vision) VISION="$2"; shift 2 ;;
    --debug-dir) DEBUG_DIR="$2"; shift 2 ;;
    --results) RESULTS="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "[sim] 未知参数：$1" >&2; exit 2 ;;
  esac
done

# 并行隔离：同机跑两套仿真时，话题名（/world/aic_warehouse/*、/control/*）完全相同，
# 真值桩会替对方的位点秒回类型 —— 表现为「类型整片错位 + 莫名 TIMEOUT」。
# 给每个实例独立的 ROS domain 与 Gazebo partition 即可互不干扰：
#   ./scripts/run_sim.sh --domain auto   # 自动挑空闲号（推荐，同机多套并行就靠它）
#   ./scripts/run_sim.sh --domain 7      # 手挑固定号（只在确认没人用时）
# 注意：手工 `ign topic` 查这个实例时要带同样的 IGN_PARTITION。
# shellcheck source=lib/sim_domain.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/sim_domain.sh"
if [[ "${DOMAIN}" == "auto" ]]; then
  DOMAIN="$(sim_alloc_domain)" || { echo "[sim] 找不到空闲 domain" >&2; exit 2; }
  echo "[sim] 自动分配 domain=${DOMAIN}"
fi
if [[ -n "${DOMAIN}" ]]; then
  sim_apply_domain "${DOMAIN}"
  echo "[sim] 隔离：ROS_DOMAIN_ID=${ROS_DOMAIN_ID} IGN_PARTITION=${IGN_PARTITION}"
  trap 'sim_release_domain' EXIT
fi

mkdir -p logs
LOG="logs/sim_launch_${STAMP}.log"
echo "[sim] GUI=${GUI} once=${ONCE} vision=${VISION} 结果 CSV=${RESULTS}"
echo "[sim] 日志：${LOG}"

# 坑：launch 不接受空值参数（malformed launch argument 'x:='），
#     debug_dir 默认空，必须「非空才追加」而不是无条件传。
EXTRA_ARGS=()
if [[ -n "${DEBUG_DIR}" ]]; then
  EXTRA_ARGS+=("vision_debug_dir:=${DEBUG_DIR}")
fi

ros2 launch aic_gazebo_sim sim_decision.launch.py \
  gui:="${GUI}" \
  once:="${ONCE}" \
  auto_restart:="${AUTORESTART}" \
  results_csv:="${RESULTS}" \
  drop_rate:="${DROP}" \
  invalid_rate:="${INVALID}" \
  vision_backend:="${VISION}" \
  "${EXTRA_ARGS[@]}" 2>&1 | tee "${LOG}"
