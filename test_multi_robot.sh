#!/bin/bash
set -e

cd /home/yy/autonomy_stack_diablo_setup
source ./install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:src/mtare_planner/tare_planner/or-tools/lib

echo "========================================="
echo "Multi-Robot MTARE Planner Test"
echo "========================================="

# Start Unity simulation
./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
UNITY_PID=$!
echo "Unity started (PID: $UNITY_PID)"
sleep 3

# Launch ROS2 system
echo ""
echo "Launching ROS2 multi-robot system..."
ros2 launch vehicle_simulator system_simulation_with_mtare_planner_multi_robot.launch.py robot_num:=2 &
LAUNCH_PID=$!
echo "Launch started (PID: $LAUNCH_PID)"

# Wait for initialization
echo ""
echo "Waiting for initialization (15s)..."
sleep 15

echo ""
echo "========================================="
echo "Checking System Status"
echo "========================================="

# Check tare_planner_node processes
echo ""
echo "1. tare_planner_node processes:"
ps aux | grep tare_planner_node | grep -v grep | awk '{print "   PID:", $2, "CPU:", $3 "%, MEM:", $4 "%"}'

# Check ROS2 nodes
echo ""
echo "2. ROS2 Nodes:"
ros2 node list 2>/dev/null | grep -E "robot_|tare" | head -10 | while read line; do
    echo "   $line"
done

# Check topics
echo ""
echo "3. Key Topics:"
ros2 topic list 2>/dev/null | grep -E "robot_.*terrain|robot_.*registered|exploration_info" | head -10 | while read line; do
    echo "   $line"
done

# Check TF
echo ""
echo "4. TF Frames:"
ros2 run tf2_ros tf2_echo map sensor 2>&1 | head -3 || echo "   TF: map -> sensor (check in progress)"

echo ""
echo "========================================="
echo "Test Complete. Press Enter to stop..."
echo "========================================="
read

# Cleanup
kill $LAUNCH_PID 2>/dev/null
kill $UNITY_PID 2>/dev/null
pkill -9 "Model.x86_64" 2>/dev/null
pkill -f "tare_planner_node" 2>/dev/null
echo "Cleanup complete."
