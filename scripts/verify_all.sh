#!/bin/bash
# 一键验收：L1 构建 -> L2 协议层 -> L3 RSC 干跑 -> L4 仿真基线 [-> L5 PLC 接管整轮]
#
# 用法：
#   ./scripts/verify_all.sh                 # L1–L4，约 3.5 分钟
#   ./scripts/verify_all.sh --with-plc      # 追加 L5（+约 3 分钟）
#   ./scripts/verify_all.sh --skip-build    # 跳过 L1（刚构建过）
#   ./scripts/verify_all.sh --only L2,L3    # 只跑指定级别
#   ./scripts/verify_all.sh --list          # 只列出各级在验什么
#
# 退出码：0 = 全过；1 = 有失败；2 = 环境或参数问题
#
# 会先 pkill gzserver 与残留网关：跑之前别开着别的仿真实例。
# 每级的完整输出在 logs/acc_<级别>.log。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

WITH_PLC=0
SKIP_BUILD=0
ONLY=""

usage() { sed -n '2,18p' "$0"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --with-plc)   WITH_PLC=1; shift ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    --only)       ONLY="${2:-}"; shift 2 ;;
    --list)
      cat <<'EOF'
  L1  构建            全部 ROS 包能编过（colcon 无 error）
  L2  协议层          Modbus TCP 从站能连、寄存器可读、心跳递增
  L3  RSC 干跑        RSC 后端整轮握手 15 项断言（不需要 vPLC）
  L4  仿真基线        真视觉 12 轮 + 三组断言（决策/夹持/落位）
  L5  PLC 接管整轮    软件主站驱动 12 块料全部入架（需 --with-plc）
EOF
      exit 0 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1（-h 看用法）" >&2; exit 2 ;;
  esac
done

want() {
  [ -z "$ONLY" ] && return 0
  case ",${ONLY}," in *",$1,"*) return 0 ;; *) return 1 ;; esac
}

# ---- 环境：顺序不能反，set -u 必须放在 source 之后 ----
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[验收] 找不到 /opt/ros/humble/setup.bash" >&2; exit 2
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ ! -f install/setup.bash ]; then
  echo "[验收] install/ 不存在。先跑 ./scripts/build.sh" >&2; exit 2
fi
# shellcheck disable=SC1091
source install/setup.bash
set -u
export ROS_HOME=$PWD/logs/.ros ROS_LOG_DIR=$PWD/logs/.ros/log ROS_LOCALHOST_ONLY=1
# 坑：ros2 run 走 #!/usr/bin/env python3，可能命中没装 cv2 的托管 3.13。
export PATH=/usr/bin:$PATH
mkdir -p logs

# ---- 跑前清场 ----
# 坑：pkill -f gzserver 在 Fortress 上匹配 0 个进程（真名是 `ign gazebo`）。
# 上一轮实例存活时会和新实例抢关节指令，表现为 RTF 掉到 1/10、落位偏移。
# shellcheck source=lib/sim_cleanup.sh
source "$ROOT/scripts/lib/sim_cleanup.sh"
sim_kill_stale

ROWS=()
FAILS=0
T_START=$SECONDS

record() {  # id name rc secs evidence
  local id="$1" name="$2" rc="$3" secs="$4" ev="$5" st
  if [ "$rc" -eq 0 ]; then st="PASS"; else st="FAIL"; FAILS=$((FAILS + 1)); fi
  # 状态放前面：中文名的显示宽度是 2 列/字，printf 按字节补齐会错列。
  printf "  %-3s %-4s %4ss  %s —— %s\n" "$id" "$st" "$secs" "$name" "$ev"
  ROWS+=("$id|$name|$st|$secs|$ev")
}

echo "================================================================"
echo " AIC 仿真 / PLC 验收   $(date '+%Y-%m-%d %H:%M:%S')"
echo " 工作区：$ROOT"
echo "================================================================"

# ---------- L1 构建 ----------
if want L1 && [ "$SKIP_BUILD" -eq 0 ]; then
  t0=$SECONDS
  ./scripts/build.sh > logs/acc_build.log 2>&1
  rc=$?
  secs=$((SECONDS - t0))
  if grep -qE "Failed +<<<|error:" logs/acc_build.log; then rc=1; fi
  pkgs=$(grep -cE "Finished +<<<" logs/acc_build.log || true)
  if [ "$rc" -eq 0 ]; then ev="${pkgs} 个包编过"; else ev="见 logs/acc_build.log"; fi
  record L1 "构建" "$rc" "$secs" "$ev"
elif want L1; then
  record L1 "构建" 0 0 "已跳过（--skip-build）"
fi

# ---------- L2 协议层 ----------
if want L2; then
  t0=$SECONDS
  PORT=5031 bash scripts/test_plc_proto.sh > logs/acc_proto.log 2>&1
  rc=$?
  secs=$((SECONDS - t0))
  hbs=$(grep -o "HB=[0-9]*" logs/acc_proto.log | cut -d= -f2 || true)
  first=$(printf '%s\n' "$hbs" | head -1)
  last=$(printf '%s\n' "$hbs" | tail -1)
  ev="心跳 ${first:-?}→${last:-?}"
  # 判定：连上 + 心跳至少涨 1（3 次采样约 0.6 s，1 Hz 心跳涨 1 属正常）
  if [ -z "$first" ] || [ -z "$last" ]; then rc=1; ev="主站没读到寄存器"; fi
  if [ "${last:-0}" -le "${first:-0}" ] && [ "$rc" -eq 0 ]; then
    rc=1; ev="心跳没递增（${first}→${last}）"
  fi
  record L2 "协议层 Modbus" "$rc" "$secs" "$ev"
fi

# ---------- L3 RSC 干跑 ----------
if want L3; then
  t0=$SECONDS
  bash scripts/test_plc_rsc.sh > logs/acc_rsc.log 2>&1
  rc=$?
  secs=$((SECONDS - t0))
  if grep -q "全部通过：RSC" logs/acc_rsc.log; then
    ok=$(grep -c "^  OK " logs/acc_rsc.log || true)
    ev="${ok} 项断言全过"
  else
    rc=1
    ev=$(grep -m1 "^失败" logs/acc_rsc.log || echo "未打印通过标记")
  fi
  record L3 "RSC 干跑整轮" "$rc" "$secs" "$ev"
fi

# ---------- L4 仿真基线 ----------
if want L4; then
  t0=$SECONDS
  ./scripts/verify_sim_m2.sh --domain auto > logs/acc_sim.log 2>&1
  rc=$?
  secs=$((SECONDS - t0))
  if [ "$rc" -eq 0 ] && grep -q "^PASS：" logs/acc_sim.log; then
    # 不用 cut -c 截断：中文是多字节，按字符截会切出半个字
    dec=$(grep -m1 "^PASS：" logs/acc_sim.log | grep -o "决策 [0-9]*/[0-9]* OK" || true)
    npl=$(grep -m1 "^PASS：" logs/acc_sim.log \
          | grep -o "[0-9]* 块铁片全部落在对应类型的格内" | cut -d' ' -f1 || true)
    ev="${dec:-决策 ?}、${npl:-?} 块全部落格"
  else
    ev=$(grep -m1 -E "^FAIL|失败现场" logs/acc_sim.log || echo "见 logs/acc_sim.log")
    rc=1
  fi
  record L4 "仿真基线 12 轮" "$rc" "$secs" "$ev"
fi

# ---------- L5 PLC 接管整轮 ----------
if [ "$WITH_PLC" -eq 1 ] && want L5; then
  t0=$SECONDS
  PORT=5031 bash scripts/test_plc_full.sh > logs/acc_plc.log 2>&1
  rc=$?
  secs=$((SECONDS - t0))
  line=$(grep -m1 "汇总：成功" logs/acc_plc.log || true)
  if [ "$rc" -eq 0 ] && printf '%s' "$line" | grep -q "成功 12/12"; then
    ev=$(printf '%s' "$line" | sed 's/^汇总：//; s/，货架.*//')
  else
    rc=1
    # 落位断言失败时，主站汇总仍是 12/12，必须把断言里的第一条抓出来当证据
    f=$(grep -m1 -E "^FAIL|^  - [ABC][0-9]" logs/acc_plc.log || true)
    if [ -n "$f" ]; then
      ev="落位/结果断言未过：$(printf '%s' "$f" | sed 's/^ *//')"
    else
      ev=$(printf '%s' "$line" | sed 's/^汇总：//; s/，货架.*//')
      [ -n "$ev" ] || ev="见 logs/acc_plc.log"
    fi
  fi
  record L5 "PLC 接管整轮" "$rc" "$secs" "$ev"
fi

# ---------- 汇总 ----------
TOTAL=$((SECONDS - T_START))
N=${#ROWS[@]}
PASSED=$((N - FAILS))
echo "----------------------------------------------------------------"
printf " 结果：%d/%d 通过，总耗时 %d 分 %02d 秒\n" \
  "$PASSED" "$N" "$((TOTAL / 60))" "$((TOTAL % 60))"
echo " 日志：logs/acc_*.log（L4 另有 logs/verify_m2_<时间戳>.{csv,log}）"
if [ "$FAILS" -gt 0 ]; then
  echo " 失败项："
  for r in "${ROWS[@]}"; do
    IFS='|' read -r id name st _ ev <<< "$r"
    [ "$st" = "FAIL" ] && echo "   - $id $name：$ev"
  done
  echo "================================================================"
  exit 1
fi
echo "================================================================"
exit 0
