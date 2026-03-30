#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./source_workspace_setup.bash

# Set or-tools library path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SCRIPT_DIR/src/mtare_planner/tare_planner/or-tools/lib

echo "=========================================="
echo "Multi-Robot MTARE Planner (No Unity Mode)"
echo "=========================================="
echo ""
echo "Starting ROS2 multi-robot system..."
echo "RViz will start automatically."
echo ""

# Launch multi-robot ROS2 system (no Unity)
ros2 launch vehicle_simulator system_simulation_with_mtare_planner_multi_robot.launch.py robot_num:=2 use_simulation:=true &
LAUNCH_PID=$!

sleep 3

echo "Starting RViz..."
ros2 run rviz2 rviz2 -d src/mtare_planner/tare_planner/rviz/tare_planner_multi_robot.rviz &
RVIZ_PID=$!

echo ""
echo "=========================================="
echo "System is running. Close RViz to stop."
echo "=========================================="

# Wait for RViz to close
wait $RVIZ_PID

# Cleanup
echo ""
echo "Shutting down..."
kill $LAUNCH_PID 2>/dev/null
pkill -f "tare_planner_node" 2>/dev/null
pkill -f "rviz2" 2>/dev/null

echo "Done!"
