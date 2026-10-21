#!/bin/bash
# 仿真相关进程的统一清理（幂等、可重复调用）。
#
# 为什么不能用 `pkill -f gzserver`：
#   Fortress 的进程名是 `ign gazebo -s -r --headless-rendering ...`，
#   `gzserver` 是 Gazebo classic 的名字。在本栈上 `pkill -f gzserver` 匹配 0 个进程，
#   **看起来执行了、实际什么都没做**——于是上一轮的实例继续跑，新实例照起，
#   两套物理世界共用同一批话题、互相抢关节指令，表现为 RTF 掉到十分之一、
#   轴「到位超时」、落位偏移。这个假象排查了很久，记在 acceptance_manual.md §10。

SIM_CLEAN_PATTERNS=(
  "ign gazebo"                              # gzserver / gzclient / gui
  "lib/aic_gazebo_sim/gantry_controller"    # 运动控制
  "lib/aic_gazebo_sim/sim_vision_node"      # 真值桩
  "lib/aic_gazebo_sim/capture_storyboard"   # 抓帧
  "lib/aic_vision_trad/vision_node"         # 真视觉
  "lib/aic_dashboard/score_node"            # 判分看板
  "ros_gz_bridge"                           # 桥接
  "lib/aic_plc_gateway/plc_gateway"         # PLC 网关
)

# 列出残留 PID（排除当前 shell 与其父 shell，避免把自己的命令行匹配进去）
sim_pids() {
  local pat pid
  for pat in "${SIM_CLEAN_PATTERNS[@]}"; do
    for pid in $(pgrep -f "${pat}" 2>/dev/null || true); do
      [ "${pid}" = "$$" ] && continue
      [ "${pid}" = "${PPID}" ] && continue
      printf '%s\n' "${pid}"
    done
  done | sort -u
}

# 数 Gazebo 实例。用 ps + 方括号写法：`pgrep -f "ign gazebo"` 会匹配到
# 调用者自己的命令行（里面就写着这个字符串），在交互 shell 里会多数 1–2 个。
sim_count_gazebo() {
  ps -eo args= 2>/dev/null | grep -c "[i]gn gazebo" || true
}

# 清掉所有残留。返回 0 表示清理后一个不剩。
sim_kill_stale() {
  local pids pid n
  pids=$(sim_pids)
  if [ -z "${pids}" ]; then
    return 0
  fi
  n=$(printf '%s\n' "${pids}" | wc -l)
  echo "[cleanup] 发现 ${n} 个残留进程，清理中"
  for pid in ${pids}; do kill -TERM "${pid}" 2>/dev/null || true; done
  sleep 2
  for pid in ${pids}; do kill -KILL "${pid}" 2>/dev/null || true; done
  sleep 1
  pids=$(sim_pids)
  if [ -n "${pids}" ]; then
    echo "[cleanup] 仍有残留：$(printf '%s ' ${pids})" >&2
    return 1
  fi
  return 0
}

# 起跑前自检：必须 0 个 Gazebo 实例（多实例会静默污染结果）
sim_assert_single() {
  local n
  n=$(sim_count_gazebo)
  if [ "${n}" -gt 1 ]; then
    echo "[cleanup] 有 ${n} 个 Gazebo 实例在跑，先清理" >&2
    sim_kill_stale || return 1
  fi
  return 0
}
