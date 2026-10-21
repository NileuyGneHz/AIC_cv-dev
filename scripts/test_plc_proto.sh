#!/bin/bash
# 协议层自测：只起网关 + 软件主站，不需要仿真。
# 验证 Modbus TCP 从站能连、寄存器/离散输入能读、心跳在走。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u          # 必须放在 source 之后：ROS 的 setup.bash 会引用未定义变量
export ROS_HOME=$PWD/logs/.ros ROS_LOG_DIR=$PWD/logs/.ros/log ROS_LOCALHOST_ONLY=1

PORT="${PORT:-5021}"
ros2 launch aic_plc_gateway plc_gateway.launch.py port:="$PORT" > logs/plc_gw.log 2>&1 &
GW=$!
trap 'kill -INT $GW 2>/dev/null' EXIT
sleep 5

echo "=== 网关日志 ==="
tail -4 logs/plc_gw.log
echo "=== 主站读 3 次（observe） ==="
/usr/bin/python3 "$ROOT/src/plugins/aic_plc_gateway/scripts/plc_master_demo.py" \
  --mode observe --port "$PORT" --loops 3 --interval 0.3
