#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR" || exit 1
source ./source_workspace_setup.bash

NO_RVIZ=0
SCENE_PROFILE="${WHITEBOX_SCENE_PROFILE:-realistic}"
EXPLORATION_CONFIG="${EXPLORATION_PLANNER_CONFIG:-indoor}"
BOUNDARY_FILE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz)
      NO_RVIZ=1
      shift
      ;;
    --scene-profile)
      if [[ $# -lt 2 ]]; then
        echo "--scene-profile requires one value: realistic, compact, or office"
        exit 1
      fi
      SCENE_PROFILE="$2"
      shift 2
      ;;
    --config)
      if [[ $# -lt 2 ]]; then
        echo "--config requires one exploration planner scenario name"
        exit 1
      fi
      EXPLORATION_CONFIG="$2"
      shift 2
      ;;
    --boundary-file)
      if [[ $# -lt 2 ]]; then
        echo "--boundary-file requires one PLY path"
        exit 1
      fi
      BOUNDARY_FILE="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--no-rviz] [--scene-profile realistic|compact|office] [--config indoor] [--boundary-file PATH]"
      exit 1
      ;;
  esac
done

if [[ "$SCENE_PROFILE" != "realistic" && "$SCENE_PROFILE" != "compact" && "$SCENE_PROFILE" != "office" ]]; then
  echo "Invalid scene profile: $SCENE_PROFILE"
  echo "Expected: realistic, compact, or office"
  exit 1
fi

cleanup() {
  if [[ -n "${WHITEBOX_TERRAIN_PID:-}" ]]; then
    kill "$WHITEBOX_TERRAIN_PID" 2>/dev/null || true
  fi
  if [[ -n "${TARE_LAUNCH_PID:-}" ]]; then
    kill "$TARE_LAUNCH_PID" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
  fi
  cleanup_stale_sim_processes
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
  kill_by_pattern "scripts/publish_whitebox_vehicle_terrain_map.py"
  kill_by_pattern "scripts/whitebox_stair_goal_router.py"
  kill_by_pattern "system_simulation_with_exploration_planner_gazebo.launch.py"
  kill_by_pattern "system_simulation_with_exploration_planner_ns.launch.py"
  kill_by_pattern "system_simulation_with_exploration_planner.launch"
  kill_by_pattern "system_simulation_with_route_planner_gazebo.launch.py"
  kill_by_pattern "install/vehicle_simulator/lib/vehicle_simulator/registeredScanFromOdom"
  kill_by_pattern "install/vehicle_simulator/lib/vehicle_simulator/vehicleSimulator"
  kill_by_pattern "install/vehicle_simulator/lib/vehicle_simulator/poseStampedToGazeboSetPose"
  kill_by_pattern "install/visualization_tools/lib/visualization_tools/visualizationTools"
  kill_by_pattern "install/local_planner/lib/local_planner/localPlanner"
  kill_by_pattern "install/local_planner/lib/local_planner/pathFollower"
  kill_by_pattern "install/terrain_analysis/lib/terrain_analysis/terrainAnalysis"
  kill_by_pattern "install/terrain_analysis_ext/lib/terrain_analysis_ext/terrainAnalysisExt"
  kill_by_pattern "install/sensor_scan_generation/lib/sensor_scan_generation/sensorScanGeneration"
  kill_by_pattern "install/tare_planner/lib/tare_planner/tare_planner_node"
  kill_by_pattern "install/tare_planner/lib/tare_planner/navigationBoundary"
  kill_by_pattern "install/ros_gz_bridge/lib/ros_gz_bridge/parameter_bridge"
  kill_by_pattern "ros_gz_bridge/parameter_bridge"
  kill_by_pattern "ign gazebo"
  kill_by_pattern "rviz2"
  sleep 1
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
}

trap cleanup EXIT

cleanup_stale_sim_processes
VEHICLE_X="0.0"
VEHICLE_Y="0.0"
VEHICLE_TERRAIN_MAP_TOPIC="/terrain_map"
if [[ "$SCENE_PROFILE" == "office" ]]; then
  python3 scripts/generate_unity_office_gazebo_scene.py >/dev/null
  WORLD_NAME="unity_office_gazebo"
  SCENE_MESH_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity_office_gazebo/unity_office_gazebo.obj"
  SCENE_MAP_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity_office_gazebo/map.ply"
  SCENE_METADATA_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity_office_gazebo/unity_office_gazebo.json"
  if [[ -z "$BOUNDARY_FILE" ]]; then
    BOUNDARY_FILE="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity_office_gazebo/boundary.ply"
  fi
else
  python3 scripts/generate_whitebox_stair_test_scene.py --profile "$SCENE_PROFILE" >/dev/null
  WORLD_NAME="whitebox_stair_test"
  SCENE_MESH_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.obj"
  SCENE_MAP_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/map.ply"
  SCENE_METADATA_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.json"
  VEHICLE_X="-5.0"
  VEHICLE_Y="-1.8"
  VEHICLE_TERRAIN_MAP_TOPIC="/whitebox_vehicle_terrain_map"
  if [[ -z "$BOUNDARY_FILE" ]]; then
    BOUNDARY_FILE="$SCRIPT_DIR/src/exploration_planner/tare_planner/data/whitebox_36m_boundary.ply"
  fi
fi

if [[ "$VEHICLE_TERRAIN_MAP_TOPIC" == "/whitebox_vehicle_terrain_map" ]]; then
  python3 scripts/publish_whitebox_vehicle_terrain_map.py \
    --metadata "$SCENE_METADATA_PATH" &
  WHITEBOX_TERRAIN_PID=$!
fi

wait_for_nonzero_clock() {
  local deadline=$((SECONDS + 30))
  while [[ "$SECONDS" -lt "$deadline" ]]; do
    local sec
    sec=$(timeout 2s ros2 topic echo /clock --once --field clock.sec 2>/dev/null | tr -dc '0-9' || true)
    if [[ -n "$sec" && "$sec" -gt 0 ]]; then
      sleep 1
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for nonzero /clock before starting TARE."
  return 1
}

ros2 launch vehicle_simulator system_simulation_with_exploration_planner_gazebo.launch.py \
  exploration_planner_config:="$EXPLORATION_CONFIG" \
  world_name:="$WORLD_NAME" \
  vehicleX:="$VEHICLE_X" \
  vehicleY:="$VEHICLE_Y" \
  vehicleTerrainMapTopic:="$VEHICLE_TERRAIN_MAP_TOPIC" \
  scene_mesh_path:="$SCENE_MESH_PATH" \
  scene_map_path:="$SCENE_MAP_PATH" \
  boundary_file:="$BOUNDARY_FILE" \
  launch_tare:=false \
  gazebo_gui:=$([[ "$NO_RVIZ" -eq 1 ]] && echo false || echo true) &
LAUNCH_PID=$!

wait_for_nonzero_clock
ros2 launch tare_planner explore_world.launch \
  use_sim_time:=true \
  scenario:="$EXPLORATION_CONFIG" \
  boundary_file:="$BOUNDARY_FILE" &
TARE_LAUNCH_PID=$!

sleep 1

if [[ "$NO_RVIZ" -eq 1 ]]; then
  wait "$LAUNCH_PID"
else
  ros2 run rviz2 rviz2 -d src/exploration_planner/tare_planner/rviz/tare_planner_ground.rviz \
    --ros-args -p use_sim_time:=true
fi
