#!/bin/bash
echo "Killing all ROS2 and Unity processes..."

# Kill ROS2 related processes
pkill -9 "ros2" 2>/dev/null
pkill -9 "rviz2" 2>/dev/null
pkill -f "tare_planner_node" 2>/dev/null
pkill -f "exploration_planner" 2>/dev/null
pkill -f "vehicleSimulator" 2>/dev/null
pkill -f "localPlanner" 2>/dev/null
pkill -f "pathFollower" 2>/dev/null
pkill -f "terrainAnalysis" 2>/dev/null
pkill -f "sensorScanGeneration" 2>/dev/null
pkill -f "visualizationTools" 2>/dev/null
pkill -f "default_server_endpoint" 2>/dev/null
pkill -9 "static_transform_publisher" 2>/dev/null
pkill -9 "joy_node" 2>/dev/null
pkill -f "navigationBoundary" 2>/dev/null
pkill -f "sim_image_repub" 2>/dev/null

# Kill Unity
pkill -9 "Model.x86_64" 2>/dev/null
pkill -9 "environment" 2>/dev/null

# Kill any remaining python ROS nodes
pkill -f "ros_tcp_endpoint" 2>/dev/null

echo "Done!"
echo ""
echo "Remaining processes (if any):"
ps aux | grep -E "ros2|rviz|tare_planner|vehicleSimulator|Model.x86_64" | grep -v grep | grep -v "kill_all.sh" || echo "None - All cleaned up!"
