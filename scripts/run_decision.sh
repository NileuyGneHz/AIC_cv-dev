#!/usr/bin/env bash
# 启动决策节点（可选带假 vision，真 vision 未就绪时用）。
#
# 用法：
#   ./scripts/run_decision.sh                     # 只起决策
#   ./scripts/run_decision.sh --once              # 跑完一轮就退出
#   ./scripts/run_decision.sh --with-fake-vision  # 同时起假 vision（按 1,2,3,4 循环应答）
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

if [[ ! -f install/setup.bash ]]; then
  echo "[run] 还没有 install/，先执行 ./scripts/build.sh" >&2
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

# 坑：rcl 初始化日志时要写 ~/.ros/log；HOME 不可写时节点会直接报
#     "failed to initialize logging: Failed to get logging directory" 起不来。
export ROS_HOME="${ROS_HOME:-${WS_DIR}/logs/.ros}"
export ROS_LOG_DIR="${ROS_HOME}/log"
mkdir -p "${ROS_LOG_DIR}"

ONCE="false"
WITH_FAKE="0"
for arg in "$@"; do
  case "${arg}" in
    --once) ONCE="true" ;;
    --with-fake-vision) WITH_FAKE="1" ;;
    *) echo "[run] 未知参数：${arg}" >&2; exit 2 ;;
  esac
done

mkdir -p logs
STAMP="$(date +%Y%m%d_%H%M%S)"
RESULTS_CSV="logs/results_${STAMP}.csv"

if [[ "${WITH_FAKE}" == "1" ]]; then
  /usr/bin/python3 scripts/fake_vision_pub.py --mode sequence &
  FAKE_PID=$!
  trap 'kill ${FAKE_PID} 2>/dev/null || true' EXIT
  echo "[run] 假 vision 已起（pid ${FAKE_PID}）"
fi

echo "[run] 结果 CSV：${RESULTS_CSV}"
ros2 run aic_behavior_tree bt_runner --ros-args \
  -p once:="${ONCE}" \
  -p results_csv:="${RESULTS_CSV}"
