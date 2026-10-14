#!/usr/bin/env bash
# 决策闭环自验：假 vision 应答 + 话题记录 + 结果断言，一次跑两个场景。
#
#   场景 A（正常）  ：12 个位点全部识别成功 -> 12 条 GOTO + 12 条 HANDLE，结果表 12 行 OK
#   场景 B（抗扰动）：每第 5 条 vision 回复发非法值 -> 12 条 GOTO + 10 条 HANDLE，
#                     两个位点记为 TIMEOUT，且巡视不中断（证明兜底分支有效）
#
# 用法：./scripts/verify_decision_loop.sh
# 产物：logs/verify_A_*.csv|log、logs/verify_B_*.csv|log
set -uo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

if [[ ! -f install/setup.bash ]]; then
  echo "[verify] 还没有 install/，先执行 ./scripts/build.sh" >&2
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
# 本机自验只在回环上通信，避免多机/多网卡环境下的发现失败干扰结论
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

# 坑：rcl 初始化日志时要写 ~/.ros/log。本机 HOME 不可写时会直接报
#     "failed to initialize logging: Failed to get logging directory"，
#     进程根本起不来（表现成「节点莫名启动失败」）。把 ROS_HOME 落到工作区内。
export ROS_HOME="${ROS_HOME:-${WS_DIR}/logs/.ros}"
export ROS_LOG_DIR="${ROS_HOME}/log"
mkdir -p "${ROS_LOG_DIR}"

mkdir -p logs
PY=/usr/bin/python3
OVERALL=0

# $1=场景名 $2=invalid_every $3=expect_handle $4=expect_timeout $5=expect_types $6=vision_timeout
run_scenario() {
  local name="$1" invalid_every="$2" expect_handle="$3" expect_timeout="$4"
  local expect_types="$5" vt="$6"
  local base="logs/verify_${name}"
  local rc_runner=0

  echo ""
  echo "=============== 场景 ${name}：invalid_every=${invalid_every} 期望 HANDLE=${expect_handle} TIMEOUT=${expect_timeout} ==============="
  rm -f "${base}"_*.csv "${base}".log "${base}"_*.log

  # 记录器（先起，避免漏掉最早的 GOTO）
  "${PY}" scripts/record_task_topic.py \
    --out "${base}_tasks.csv" --expect-handle "${expect_handle}" --duration 120 \
    > "${base}_recorder.log" 2>&1 &
  local rec_pid=$!
  sleep 2

  # 假 vision
  "${PY}" scripts/fake_vision_pub.py \
    --invalid-every "${invalid_every}" > "${base}_fake.log" 2>&1 &
  local fake_pid=$!
  sleep 2

  # 决策：跑完一轮就退出
  timeout 120 ros2 run aic_behavior_tree bt_runner --ros-args \
    -p once:=true \
    -p results_csv:="${base}_results.csv" \
    -p vision_timeout_s:="${vt}" > "${base}.log" 2>&1
  rc_runner=$?

  wait "${rec_pid}"
  kill "${fake_pid}" 2>/dev/null || true
  wait "${fake_pid}" 2>/dev/null || true

  echo "[verify] 决策进程退出码：${rc_runner}（0 = 一轮正常结束）"
  echo "[verify] 决策日志摘要："
  grep -E "第 .*轮结束|初始化完成|RESULT|TIMEOUT|非法" "${base}.log" | tail -20
  echo "[verify] 记录器摘要："
  tail -8 "${base}_recorder.log"

  echo "[verify] 断言："
  "${PY}" scripts/check_decision_run.py \
    --results "${base}_results.csv" \
    --tasks "${base}_tasks.csv" \
    --expect-handle "${expect_handle}" \
    --expect-timeout "${expect_timeout}" \
    --expect-types "${expect_types}"
  local rc_check=$?

  if [[ ${rc_runner} -ne 0 || ${rc_check} -ne 0 ]]; then
    echo "[verify] 场景 ${name}：FAIL"
    OVERALL=1
  else
    echo "[verify] 场景 ${name}：PASS"
  fi
}

# 期望的物料类型序列（假 vision 按 1,2,3,4 循环应答）
TYPES_ALL="1,2,3,4,1,2,3,4,1,2,3,4"
# 场景 B：第 5、10 次应答是非法值 "9" -> 这两个位点识别失败，类型按 0 记
TYPES_WITH_INVALID="1,2,3,4,0,1,2,3,4,0,1,2"

run_scenario A 0 12 0 "${TYPES_ALL}" 3.0
run_scenario B 5 10 2 "${TYPES_WITH_INVALID}" 0.6

echo ""
if [[ ${OVERALL} -eq 0 ]]; then
  echo "==================== 自验结论：PASS ===================="
else
  echo "==================== 自验结论：FAIL ===================="
fi
exit "${OVERALL}"
