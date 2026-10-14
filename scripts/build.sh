#!/usr/bin/env bash
# 构建决策侧工作区（宿主机直接编译，不需要 docker / 仿真环境）。
#
# 用法：
#   ./scripts/build.sh                # 增量构建全部包
#   ./scripts/build.sh --clean        # 先删 build/install/log 再全量构建
#   ./scripts/build.sh --packages-select aic_behavior_tree
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[build] 找不到 ROS 环境脚本：${ROS_SETUP}（可用 ROS_SETUP=... 覆盖）" >&2
  exit 2
fi

# 坑：本机 /tmp 是小容量 tmpfs（实测仅 10 MB），而 gcc 默认把中间汇编 .s 写到 /tmp，
#     模板稍多的源文件就会报 "error writing to /tmp/ccxxxx.s: No space left on device"。
#     实测给 gcc 设 TMPDIR **不生效**（仍写 /tmp）；用 -pipe 让 cc1/as 走管道、不落中间文件才是解法。
CMAKE_ARGS=(-DCMAKE_CXX_FLAGS=-pipe)

if [[ "${1:-}" == "--clean" ]]; then
  echo "[build] 清理 build/ install/ log/"
  rm -rf "${WS_DIR}/build" "${WS_DIR}/install" "${WS_DIR}/log"
  shift
fi

# 坑：ROS 的 setup.bash 里会引用未定义变量，与 set -u 冲突，source 期间必须先关掉
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

# 坑：本机 PATH 里可能有别的 python3（如 3.13），colcon / ament 需要系统 3.10。
export PATH="/usr/bin:${PATH}"
export HOME="${HOME:-/home/nileuy}"   # 沙箱/非常规 shell 下 HOME 可能指向不可写目录

cd "${WS_DIR}"
echo "[build] 工作区：${WS_DIR}"
colcon build --symlink-install --cmake-args "${CMAKE_ARGS[@]}" "$@"
echo "[build] 完成。下一步：source install/setup.bash"
