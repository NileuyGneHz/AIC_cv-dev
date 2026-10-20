#!/usr/bin/env bash
# 仿真实例的 ROS domain / Gazebo partition 分配（被 run_sim.sh 与 verify_sim_m2.sh source）。
#
# 为什么需要它：两套仿真的 ROS 话题名（/control/*、/vision/*、/world/aic_warehouse/*）
# 完全一样，共用 domain 时会互相消费对方的消息 —— 真值桩替对方位点秒回类型，
# 表现为「类型整片错位 + 莫名 TIMEOUT」，看起来极像视觉退化。
#
# 隔离靠两把锁，缺一不可：
#   ROS_DOMAIN_ID   —— ROS 2 节点/桥之间不串
#   IGN_PARTITION   —— Gazebo transport 侧不串（实测有效：空分区 `ign topic -l` 返回 0 条）
#
# 坑：不要手挑固定数字。两个人（或两个会话）都选 7 时照样撞车 ——
#     实测两个 gzserver 的 IGN_PARTITION 都是 aic_sim_7，同一分区里出现 53 条话题
#     （= 两套 × 26 条），跟「分区没生效」长得一模一样。所以这里改成自动分配。
#
# 用法（脚本里）：
#   source "$(dirname "${BASH_SOURCE[0]}")/lib/sim_domain.sh"
#   d="$(sim_alloc_domain)" && sim_apply_domain "${d}"

SIM_DOMAIN_REGISTRY="${SIM_DOMAIN_REGISTRY:-logs/.sim_domains}"
SIM_DOMAIN_LOCK="${SIM_DOMAIN_LOCK:-logs/.sim_domains.lock}"
SIM_DOMAIN_MAX="${SIM_DOMAIN_MAX:-200}"

# 扫描所有进程的 environ，列出已被占用的 ROS domain（不依赖 ros2 CLI，避免慢发现）
sim_used_domains() {
  /usr/bin/python3 - <<'PY'
import glob

used = set()
for path in glob.glob('/proc/[0-9]*/environ'):
    try:
        with open(path, 'rb') as fh:
            blob = fh.read()
    except OSError:
        continue  # 内核线程 / 别的用户的进程读不到，跳过
    for kv in blob.split(b'\0'):
        if kv.startswith(b'ROS_DOMAIN_ID='):
            try:
                used.add(int(kv.split(b'=', 1)[1]))
            except ValueError:
                pass
            break
print(' '.join(str(d) for d in sorted(used)))
PY
}

# 登记表里仍存活的 domain（补上「刚启动、environ 还没导出」的那几秒空窗）
sim_registry_domains() {
  [[ -f "${SIM_DOMAIN_REGISTRY}" ]] || return 0
  local d pid
  while read -r d pid; do
    [[ -n "${d:-}" && -n "${pid:-}" ]] || continue
    kill -0 "${pid}" 2>/dev/null && printf '%s ' "${d}"
  done < "${SIM_DOMAIN_REGISTRY}"
}

sim_prune_registry() {
  [[ -f "${SIM_DOMAIN_REGISTRY}" ]] || return 0
  local tmp d pid
  tmp="$(mktemp)"
  while read -r d pid; do
    [[ -n "${d:-}" && -n "${pid:-}" ]] || continue
    kill -0 "${pid}" 2>/dev/null && printf '%s %s\n' "${d}" "${pid}" >> "${tmp}"
  done < "${SIM_DOMAIN_REGISTRY}"
  mv "${tmp}" "${SIM_DOMAIN_REGISTRY}"
}

# 找一个真正空着的 domain：登记表/进程表都说没人用，且 Gazebo 侧该分区确实 0 条话题
sim_pick_domain() {
  local used d n
  used=" $(sim_used_domains) $(sim_registry_domains) "
  for ((d = 1; d <= SIM_DOMAIN_MAX; d++)); do
    [[ "${used}" == *" ${d} "* ]] && continue
    # 分区名不怕重（重了就是撞车），这里以 Gazebo transport 的实际可见话题数为准
    n="$(IGN_PARTITION="aic_sim_${d}" timeout 10 ign topic -l 2>/dev/null | wc -l)"
    if [[ "${n:-1}" -eq 0 ]]; then
      printf '%s\n' "${d}"
      return 0
    fi
  done
  return 1
}

# 分配 + 登记（全程持锁，避免同一瞬间启动的两个实例拿到同一个号）
sim_alloc_domain() {
  mkdir -p "$(dirname "${SIM_DOMAIN_REGISTRY}")"
  (
    exec 9>"${SIM_DOMAIN_LOCK}"
    flock 9
    sim_prune_registry
    local d
    d="$(sim_pick_domain)" || exit 1
    printf '%s %s\n' "${d}" "$$" >> "${SIM_DOMAIN_REGISTRY}"
    printf '%s\n' "${d}"
  )
}

sim_apply_domain() {
  local d="$1"
  export ROS_DOMAIN_ID="${d}"
  export IGN_PARTITION="aic_sim_${d}"
}

# 退出时注销自己的登记（不清也没事，下次分配会按 PID 剪掉）
sim_release_domain() {
  [[ -f "${SIM_DOMAIN_REGISTRY}" ]] || return 0
  local tmp
  tmp="$(mktemp)"
  awk -v me="$$" '$2 != me' "${SIM_DOMAIN_REGISTRY}" > "${tmp}" 2>/dev/null || true
  mv "${tmp}" "${SIM_DOMAIN_REGISTRY}"
}
