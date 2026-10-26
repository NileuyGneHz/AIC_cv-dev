#!/usr/bin/env bash
# 图案 / 判据联调一把梭：起仿真（只要世界 + 控制 + 相机）→ 12 块料逐块拍俯视特写
# → 打印逐帧特征与判型。
#
# 用法：
#   ./scripts/probe_patterns.sh                    # 输出 logs/patterns，拍完自动分析
#   ./scripts/probe_patterns.sh --out logs/p2      # 换输出目录
#   ./scripts/probe_patterns.sh --keep             # 拍完不停仿真（自己接着看 GUI）
#
# 什么时候用：改了 gen_world.defect_visual（图案）或 defect_classifier 的阈值之后，
# 先跑这个 30 秒级的自检，再去跑 3 分钟的 L4 —— 整轮日志看不出「单块图案长什么样」。
set -uo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

OUT="logs/patterns"
KEEP=0
SETTLE=1.6
while [[ $# -gt 0 ]]; do
    case "$1" in
        --out) OUT="$2"; shift 2 ;;
        --settle) SETTLE="$2"; shift 2 ;;
        --keep) KEEP=1; shift ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

[[ -f install/setup.bash ]] || { echo "[probe] 还没有 install/，先 ./scripts/build.sh" >&2; exit 2; }
# ROS 的 setup.bash 会引用未定义变量，和 set -u 冲突：source 期间先关掉
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u
export PATH=/usr/bin:$PATH
export HOME="${HOME:-/home/nileuy}"
export ROS_HOME="${WS_DIR}/logs/.ros"
export ROS_LOG_DIR="${WS_DIR}/logs/.ros/log"
export ROS_LOCALHOST_ONLY=1
# 本机 ign-physics 引擎库不在默认搜索路径（见 simulation_guide 坑 1）
PHYS="$(ls -d /usr/lib/x86_64-linux-gnu/ign-physics-*/engine-plugins 2>/dev/null | head -1 || true)"
[[ -n "${PHYS}" ]] && export IGN_PHYSICS_ENGINE_PATH="${PHYS}"

# shellcheck disable=SC1091
source scripts/lib/sim_cleanup.sh
sim_kill_stale >/dev/null 2>&1

LOG="logs/probe_patterns.log"
ros2 launch aic_gazebo_sim sim.launch.py > "${LOG}" 2>&1 &
LP=$!

cleanup() {
    if [[ "${KEEP}" -eq 0 ]]; then
        kill -INT "${LP}" 2>/dev/null || true
        sleep 2
        sim_kill_stale >/dev/null 2>&1
    fi
}
trap cleanup EXIT

echo "[probe] 等仿真就绪（/camera/image）..."
for _ in $(seq 1 40); do
    if grep -q "就绪\|Ready" "${LOG}" 2>/dev/null && ign topic -l 2>/dev/null | grep -q "^/camera/image$"; then
        break
    fi
    sleep 1
done
sleep 2

/usr/bin/python3 scripts/probe_patterns.py --out "${OUT}" --settle "${SETTLE}"
RC=$?

echo "[probe] 逐帧特征与判型 ------------------------------------------------"
/usr/bin/python3 scripts/analyze_patterns.py "${OUT}"
exit ${RC}
