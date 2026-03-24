#!/bin/bash
# Multi-robot simulation with MTARE planner
# Supports two modes:
#   1. Shared Unity (single instance, requires Unity to support multi-robot) - NOT IMPLEMENTED YET
#   2. Separate Unity instances (one per robot) - RECOMMENDED

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd $SCRIPT_DIR

# Setup environment
export LD_LIBRARY_PATH=$SCRIPT_DIR/src/mtare_planner/tare_planner/or-tools/lib:$LD_LIBRARY_PATH
source ./install/setup.bash

ROBOT_NUM=${1:-2}
UNITY_MODE=${2:-"separate"}  # "shared" or "separate"

echo "============================================"
echo "Multi-Robot Simulation with MTARE Planner"
echo "============================================"
echo "Robot number: $ROBOT_NUM"
echo "Unity mode: $UNITY_MODE"
echo ""

if [ "$UNITY_MODE" == "shared" ]; then
    echo "WARNING: Shared Unity mode requires Unity to support multi-port connections."
    echo "Current Unity build uses hardcoded port 10000."
    echo "Please use 'separate' mode or rebuild Unity with multi-robot support."
    echo ""
    echo "Press Ctrl+C to cancel, or wait 5 seconds to continue..."
    sleep 5
fi

# Unity paths
UNITY_BASE_DIR="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity/environment"

# Check if Unity exists
if [ ! -f "$UNITY_BASE_DIR/Model.x86_64" ]; then
    echo "ERROR: Unity executable not found at $UNITY_BASE_DIR/Model.x86_64"
    exit 1
fi

echo "Starting $ROBOT_NUM robots..."

# Kill existing processes
pkill -9 "Model.x86_64" 2>/dev/null || true
sleep 1

if [ "$UNITY_MODE" == "separate" ]; then
    echo ""
    echo "Mode: Separate Unity instances (one per robot)"
    echo "Each Unity instance uses port 10000 internally"
    echo "TCP Endpoints use ports 10000, 10001, ..."
    echo ""
    echo "IMPORTANT: You need $ROBOT_NUM separate Unity builds with different ports."
    echo "Current single build only supports one robot."
    echo ""
    echo "To test with single robot, run:"
    echo "  ./system_simulation_with_mtare_planner.sh"
    echo ""
    echo "Starting ROS2 nodes only (without Unity)..."
    echo "Please manually start Unity instances if configured."
    echo ""
    
    # Start only ROS2 nodes, Unity should be started separately
    ros2 launch vehicle_simulator system_simulation_with_mtare_planner_multi_robot.launch.py \
        robot_num:=$ROBOT_NUM &
    
else
    echo ""
    echo "Mode: Shared Unity (single instance)"
    echo "Starting single Unity instance on port 10000..."
    
    # Start Unity
    "$UNITY_BASE_DIR/Model.x86_64" &
    sleep 3
    
    # Start ROS2 nodes
    ros2 launch vehicle_simulator system_simulation_with_mtare_planner_multi_robot.launch.py \
        robot_num:=$ROBOT_NUM &
fi

sleep 2

# Start RViz
echo ""
echo "Starting RViz..."
ros2 run rviz2 rviz2 -d src/mtare_planner/tare_planner/rviz/tare_planner_multi_robot.rviz &

echo ""
echo "============================================"
echo "System started!"
echo "============================================"
echo ""
echo "Robots:"
for ((i=0; i<ROBOT_NUM; i++)); do
    echo "  Robot $i: /robot_$i/ (TCP port $((10000 + i)))"
done
echo ""
echo "Topics:"
echo "  /robot_X/registered_scan    - LiDAR data"
echo "  /robot_X/state_estimation   - Odometry"
echo "  /robot_X/vehicleCommand     - Control commands"
echo "  /wheeledX/exploration_info  - Coordination (global)"
echo ""
echo "Press Enter to stop all processes..."
read

# Cleanup
echo "Stopping..."
pkill -f "tare_planner_node"
pkill -f "vehicleSimulator"
pkill -f "localPlanner"
pkill -f "terrainAnalysis"
pkill -9 "Model.x86_64" 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true

echo "Done!"
