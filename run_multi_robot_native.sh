#!/bin/bash
# 原生多机器人启动（不用 Docker）
# 用法: ./run_multi_robot_native.sh <robot_id> <robot_num>

ROBOT_ID=${1:-0}
ROBOT_NUM=${2:-2}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

export LD_LIBRARY_PATH=$SCRIPT_DIR/src/mtare_planner/tare_planner/or-tools/lib:$LD_LIBRARY_PATH
source ./source_workspace_setup.bash

echo "Starting Robot $ROBOT_ID of $ROBOT_NUM..."

# 启动 Unity
echo "[Robot $ROBOT_ID] Starting Unity..."
"$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64" &
UNITY_PID=$!
sleep 5

# 启动机器人
echo "[Robot $ROBOT_ID] Starting planner..."
ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch.py \
    robot_id:=$ROBOT_ID \
    kRobotNum:=$ROBOT_NUM \
    kAutoStart:=true \
    coordination:=true

# 清理
kill $UNITY_PID 2>/dev/null || true
