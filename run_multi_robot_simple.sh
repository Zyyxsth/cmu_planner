#!/bin/bash
# 简单多机器人方案：一个 Unity + 两个规划器（第二个没有视觉）
# 用于快速测试算法

ROBOT_NUM=${1:-2}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

export LD_LIBRARY_PATH=$SCRIPT_DIR/src/mtare_planner/tare_planner/or-tools/lib:$LD_LIBRARY_PATH
source ./source_workspace_setup.bash

echo "============================================"
echo "Multi-Robot Test (Simple Mode)"
echo "============================================"
echo "Robot 0: Full system (with Unity)"
echo "Robot 1: Planner only (no Unity)"
echo ""

# 只启动一个 Unity
echo "Starting Unity..."
"$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64" &
UNITY_PID=$!
sleep 5

# 启动 Robot 0
echo "Starting Robot 0..."
ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch.py \
    robot_id:=0 kRobotNum:=$ROBOT_NUM kAutoStart:=true coordination:=true &
ROBOT0_PID=$!

sleep 5

# 启动 Robot 1 (没有 Unity)
echo "Starting Robot 1 (no Unity)..."
ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch.py \
    robot_id:=1 kRobotNum:=$ROBOT_NUM kAutoStart:=true coordination:=true &
ROBOT1_PID=$!

# 启动 RViz
echo "Starting RViz..."
ros2 run rviz2 rviz2 -d src/mtare_planner/tare_planner/rviz/tare_planner_multi_robot.rviz &
RVIZ_PID=$!

echo ""
echo "Press Enter to stop..."
read

kill $ROBOT0_PID $ROBOT1_PID $RVIZ_PID $UNITY_PID 2>/dev/null || true
pkill -9 "Model.x86_64" 2>/dev/null || true
./kill_all.sh 2>/dev/null || true
