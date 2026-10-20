#!/usr/bin/env bash
# 缺陷几何随机化回归：逐种子重新生成世界 -> 跑一轮完整搬运 -> 汇总通过率。
#
# 为什么要有这个：M2 的视觉阈值是在「每类只有一种形状/位置」的 12 张图上标的，
# 等于测了 12 次同一个样本。这里按种子给每块铁片换缺陷的朝向/盘内位置/尺寸
# （类型不变），再看真视觉 + 决策 + 抓放是否还能整轮走通 —— 这才是阈值的真实余量。
#
# 用法：
#   ./scripts/verify_sim_robust.sh                # 默认 seed 1 2 3 4（每个约 95 s）
#   ./scripts/verify_sim_robust.sh 5 6 7          # 指定种子
#   ./scripts/verify_sim_robust.sh --vision gt 1  # 真值桩（排除视觉，只看随机几何下的物理）
#   ./scripts/verify_sim_robust.sh --frames 5     # 同时存调试图到 frames_seed_5/（做图像集/特征统计）
#
# 产物：logs/robust_<stamp>/seed_<N>.{csv,log}，末尾打印汇总表。
# 每个种子都通过 verify_sim_m2.sh --domain auto 起隔离实例，所以同机有别的仿真在跑也能跑。
# 结束（含 Ctrl-C / 失败）时会把世界恢复成 materials.jitter_seed（默认 0）的基线，
# 免得把某个随机种子留在工作区的 worlds/aic_warehouse.sdf 里。
#
# 坑：本脚本跑着的时候不要去改 verify_sim_m2.sh —— bash 是边读边执行的，
#     文件被改写会让正在跑的实例读到半截脚本（报 unexpected EOF），
#     它的 cleanup 会杀掉桥，表现为「关节反馈断流 + 视觉收不到新帧」，像仿真冻结。 
set -uo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

if [[ ! -f install/setup.bash ]]; then
  echo "[robust] 还没有 install/，先执行 ./scripts/build.sh" >&2
  exit 2
fi

VISION="trad"
FRAMES=0
SEEDS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --vision) VISION="$2"; shift 2 ;;
    --frames) FRAMES=1; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    -*) echo "[robust] 未知参数：$1" >&2; exit 2 ;;
    *) SEEDS+=("$1"); shift ;;
  esac
done
if [[ ${#SEEDS[@]} -eq 0 ]]; then
  SEEDS=(1 2 3 4)
fi

GEN="src/simulation/aic_gazebo_sim/scripts/gen_world.py"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="logs/robust_${STAMP}"
mkdir -p "${OUT}"

restore_baseline() {
  /usr/bin/python3 "${GEN}" >/dev/null 2>&1 || true
}
trap restore_baseline EXIT

printf '%-6s %-8s %-10s %-8s %s\n' "seed" "结果" "决策OK" "总耗时" "备注"
printf '%s\n' "------------------------------------------------------------"

RC_ALL=0
for seed in "${SEEDS[@]}"; do
  log="${OUT}/seed_${seed}.log"
  /usr/bin/python3 "${GEN}" --seed "${seed}" >"${OUT}/seed_${seed}.gen.txt" 2>&1
  extra=()
  if [[ ${FRAMES} -eq 1 ]]; then
    extra=(--debug-dir "${OUT}/frames_seed_${seed}")
  fi
  # verify_sim_m2 自己会写一份带时间戳的 CSV；跑完把最新那份搬进本次目录
  # --domain auto：同机常有别的仿真在跑，不隔离会直接 exit 3（或更糟——串台跑出错结论）
  ./scripts/verify_sim_m2.sh --vision "${VISION}" --timeout 300 --domain auto "${extra[@]}" >"${log}" 2>&1
  rc=$?
  csv="$(sed -n 's/^\[verify\] .*结果 CSV=//p' "${log}" | head -1)"
  ok="?"; total="?"
  if [[ -n "${csv}" && -f "${csv}" ]]; then
    cp "${csv}" "${OUT}/seed_${seed}.csv"
    # result 列是第 8 列；只数数据行
    ok="$(awk -F, 'NR>1 && $8=="OK"{n++} END{print n+0}' "${csv}")"
    total="$(awk -F, 'NR>1{n++} END{print n+0}' "${csv}")"
  fi
  if [[ ${rc} -eq 0 ]]; then
    verdict="PASS"
  else
    verdict="FAIL"
    RC_ALL=1
  fi
  # 坑：不能直接 grep 't=[0-9]+s' 后取第一个数字 —— 「释放已达 12 次」里的 12 会先被抓到
  secs="$(sed -n 's/.*释放已达 [0-9]* 次（t=\([0-9]*\)s.*/\1/p' "${log}" | head -1)"
  printf '%-6s %-8s %-10s %-8s %s\n' "${seed}" "${verdict}" "${ok}/${total}" "${secs:-?}s" "${OUT}/seed_${seed}.log"
done

printf '%s\n' "------------------------------------------------------------"
if [[ ${RC_ALL} -eq 0 ]]; then
  echo "[robust] 全部种子 PASS（${#SEEDS[@]} 轮 × 12 位点）"
else
  echo "[robust] 有种子 FAIL，先看对应日志里的「判定」与「到位超时」行" >&2
fi
echo "[robust] 产物：${OUT}/（世界已恢复为 jitter_seed 基线）"
exit ${RC_ALL}
