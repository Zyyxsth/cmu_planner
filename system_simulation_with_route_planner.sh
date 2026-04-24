#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./source_workspace_setup.bash

WHITEBOX_MODE=0
GAZEBO_MODE=0
NO_RVIZ=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gazebo)
      GAZEBO_MODE=1
      shift
      ;;
    --whitebox)
      WHITEBOX_MODE=1
      shift
      ;;
    --no-rviz)
      NO_RVIZ=1
      shift
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--gazebo] [--whitebox] [--no-rviz]"
      exit 1
      ;;
  esac
done

cleanup() {
  if [[ -n "${STATIC_SCAN_PID:-}" ]]; then
    kill "$STATIC_SCAN_PID" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
  fi
  if [[ -n "${UNITY_PID:-}" ]]; then
    kill "$UNITY_PID" 2>/dev/null || true
  fi
}

cleanup_stale_sim_processes() {
  local self_pid="$$"
  local parent_pid="$PPID"

  kill_by_pattern() {
    local pattern="$1"
    local pid
    while read -r pid; do
      if [[ -z "$pid" || "$pid" == "$self_pid" || "$pid" == "$parent_pid" ]]; then
        continue
      fi
      kill "$pid" 2>/dev/null || true
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  }

  kill_by_pattern "src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
  kill_by_pattern "scripts/publish_static_registered_scan.py"
  kill_by_pattern "system_simulation_with_route_planner_gazebo.launch.py"
  kill_by_pattern "system_simulation_with_route_planner_whitebox.launch.py"
  kill_by_pattern "system_simulation_with_route_planner.launch"
  kill_by_pattern "install/vehicle_simulator/lib/vehicle_simulator/registeredScanFromOdom"
  kill_by_pattern "install/visualization_tools/lib/visualization_tools/visualizationTools"
  kill_by_pattern "install/vehicle_simulator/lib/vehicle_simulator/vehicleSimulator"
  kill_by_pattern "install/far_planner/lib/far_planner/far_planner"
  kill_by_pattern "install/graph_decoder/lib/graph_decoder/graph_decoder"
  kill_by_pattern "install/local_planner/lib/local_planner/localPlanner"
  kill_by_pattern "install/local_planner/lib/local_planner/pathFollower"
  kill_by_pattern "install/terrain_analysis/lib/terrain_analysis/terrainAnalysis"
  kill_by_pattern "install/terrain_analysis_ext/lib/terrain_analysis_ext/terrainAnalysisExt"
  kill_by_pattern "install/sensor_scan_generation/lib/sensor_scan_generation/sensorScanGeneration"
  kill_by_pattern "install/ros_gz_bridge/lib/ros_gz_bridge/parameter_bridge"
  kill_by_pattern "ros_gz_bridge/parameter_bridge"
  kill_by_pattern "ign gazebo"
  kill_by_pattern "rviz2"
  sleep 1
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
}

trap cleanup EXIT

if [[ "$GAZEBO_MODE" -eq 1 ]]; then
  cleanup_stale_sim_processes
  python3 scripts/generate_whitebox_stair_test_scene.py >/dev/null
  SCENE_MESH_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.obj"
  SCENE_MAP_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/map.ply"
  ros2 launch vehicle_simulator system_simulation_with_route_planner_gazebo.launch.py \
    world_name:=whitebox_stair_test \
    scene_mesh_path:="$SCENE_MESH_PATH" \
    scene_map_path:="$SCENE_MAP_PATH" \
    gazebo_gui:=$([[ "$NO_RVIZ" -eq 1 ]] && echo false || echo true) &
  LAUNCH_PID=$!
elif [[ "$WHITEBOX_MODE" -eq 1 ]]; then
  python3 scripts/generate_whitebox_stair_test_scene.py >/dev/null
  python3 scripts/publish_static_registered_scan.py \
    --ply src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/map.ply &
  STATIC_SCAN_PID=$!
  sleep 1
  ros2 launch ./src/base_autonomy/vehicle_simulator/launch/system_simulation_with_route_planner_whitebox.launch.py \
    world_name:=whitebox_stair_test &
  LAUNCH_PID=$!
else
  ./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
  UNITY_PID=$!
  sleep 3
  ros2 launch vehicle_simulator system_simulation_with_route_planner.launch &
  LAUNCH_PID=$!
fi

sleep 1

if [[ "$NO_RVIZ" -eq 1 ]]; then
  wait "$LAUNCH_PID"
else
  ros2 run rviz2 rviz2 -d src/route_planner/far_planner/rviz/default.rviz
fi
