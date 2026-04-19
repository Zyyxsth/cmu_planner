#!/bin/bash

set -u

echo "Cleaning D1 upper navigation stack processes..."

# Only clean processes started from the cmu_planner upper autonomy stack.
# Do not touch /opt/d1_ros2 controller_manager / rl_controller processes.
PATTERNS=(
  "ros2 launch vehicle_simulator system_real_robot_with_route_planner_d1.launch.py"
  "ros2 launch vehicle_simulator system_real_robot_with_exploration_planner_d1.launch.py"
  "ros2 launch vehicle_simulator system_real_robot_with_mtare_planner_d1.launch.py"
  "/home/robot/cmu_planner/install/odin_ros_driver/lib/odin_ros_driver/host_sdk_sample"
  "/home/robot/cmu_planner/install/odin_autonomy_bridge/lib/odin_autonomy_bridge/odin_autonomy_bridge_node"
  "/home/robot/cmu_planner/install/d1_compat_bridge/lib/d1_compat_bridge/d1_compat_bridge_node"
  "/home/robot/cmu_planner/install/local_planner/lib/local_planner/localPlanner"
  "/home/robot/cmu_planner/install/local_planner/lib/local_planner/pathFollower"
  "/home/robot/cmu_planner/install/far_planner/lib/far_planner/far_planner"
  "/home/robot/cmu_planner/install/graph_decoder/lib/graph_decoder/graph_decoder"
  "/home/robot/cmu_planner/install/terrain_analysis/lib/terrain_analysis/terrainAnalysis"
  "/home/robot/cmu_planner/install/terrain_analysis_ext/lib/terrain_analysis_ext/terrainAnalysisExt"
  "/home/robot/cmu_planner/install/sensor_scan_generation/lib/sensor_scan_generation/sensorScanGeneration"
  "/home/robot/cmu_planner/install/visualization_tools/lib/visualization_tools/visualizationTools"
  "/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0 0 0 0 0 0 map odom"
  "/opt/ros/humble/lib/tf2_ros/static_transform_publisher -0.0 -0.0 0 0 0 0 /sensor /vehicle"
  "/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0 0 0.16 -1.5707963 0 -1.5707963 /sensor /camera"
  "/opt/ros/humble/lib/joy/joy_node --ros-args -r __node:=ps3_joy"
  "ros2 topic pub --once /goal_point"
  "ros2 topic pub -r"
  "ros2 topic echo /goal_point"
  "ros2 topic echo /path"
  "ros2 topic echo /way_point"
  "ros2 topic echo /state_estimation"
  "ros2 topic echo /d15020108/command/cmd_twist"
)

for signal in INT TERM KILL; do
  any_match=0
  for pattern in "${PATTERNS[@]}"; do
    if pgrep -af "$pattern" >/dev/null 2>&1; then
      any_match=1
      echo "[$signal] $pattern"
      pkill "-$signal" -f "$pattern" 2>/dev/null || true
    fi
  done

  if [ "$signal" != "KILL" ] && [ "$any_match" -eq 1 ]; then
    sleep 1
  fi
done

echo ""
echo "Remaining upper-stack processes:"
ps -eo pid,ppid,stat,cmd | grep -E \
  'system_real_robot_with_(route|exploration|mtare)_planner_d1.launch.py|host_sdk_sample|odin_autonomy_bridge_node|d1_compat_bridge_node|localPlanner|pathFollower|far_planner|graph_decoder|terrainAnalysis|terrainAnalysisExt|sensorScanGeneration|visualizationTools|map_to_odom_tf|vehicleTransPublisher|sensorTransPublisher|ps3_joy' \
  | grep -v grep || echo "None"
