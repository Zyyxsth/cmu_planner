#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./source_workspace_setup.bash

WHITEBOX_MODE=0
GAZEBO_MODE=0
NO_RVIZ=0
SCENE_PROFILE="${WHITEBOX_SCENE_PROFILE:-realistic}"
ROUTE_PLANNER_CONFIG=""
GOAL_TOPIC="/goal_point"

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
    --scene-profile)
      if [[ $# -lt 2 ]]; then
        echo "--scene-profile requires one value: realistic, compact, or office"
        exit 1
      fi
      SCENE_PROFILE="$2"
      shift 2
      ;;
    --route-config)
      if [[ $# -lt 2 ]]; then
        echo "--route-config requires one FAR planner config name"
        exit 1
      fi
      ROUTE_PLANNER_CONFIG="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--gazebo] [--whitebox] [--no-rviz] [--scene-profile realistic|compact|office] [--route-config CONFIG]"
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
  if [[ -n "${WHITEBOX_GOAL_ROUTER_PID:-}" ]]; then
    kill "$WHITEBOX_GOAL_ROUTER_PID" 2>/dev/null || true
  fi
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
  kill_by_pattern "scripts/publish_whitebox_vehicle_terrain_map.py"
  kill_by_pattern "scripts/whitebox_stair_goal_router.py"
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
  VEHICLE_X="0.0"
  VEHICLE_Y="0.0"
  VEHICLE_TERRAIN_MAP_TOPIC="/terrain_map"
  POSE_OVERRIDE_TOPIC=""
  if [[ "$SCENE_PROFILE" == "office" ]]; then
    python3 scripts/generate_unity_office_gazebo_scene.py >/dev/null
    WORLD_NAME="unity_office_gazebo"
    SCENE_MESH_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity_office_gazebo/unity_office_gazebo.obj"
    SCENE_MAP_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity_office_gazebo/map.ply"
    SCENE_METADATA_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity_office_gazebo/unity_office_gazebo.json"
    ROUTE_PLANNER_CONFIG="${ROUTE_PLANNER_CONFIG:-indoor}"
  else
    python3 scripts/generate_whitebox_stair_test_scene.py --profile "$SCENE_PROFILE" >/dev/null
    WORLD_NAME="whitebox_stair_test"
    SCENE_MESH_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.obj"
    SCENE_MAP_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/map.ply"
    SCENE_METADATA_PATH="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.json"
    ROUTE_PLANNER_CONFIG="${ROUTE_PLANNER_CONFIG:-whitebox_multilevel}"
    GOAL_TOPIC="/routed_goal_point"
    VEHICLE_X="-5.0"
    VEHICLE_Y="-1.8"
    VEHICLE_TERRAIN_MAP_TOPIC="/whitebox_vehicle_terrain_map"
    POSE_OVERRIDE_TOPIC="/whitebox_vehicle_pose_override"
  fi
  if [[ "$VEHICLE_TERRAIN_MAP_TOPIC" == "/whitebox_vehicle_terrain_map" ]]; then
    python3 scripts/publish_whitebox_vehicle_terrain_map.py \
      --metadata "$SCENE_METADATA_PATH" &
    WHITEBOX_TERRAIN_PID=$!
  fi
  if [[ "$GOAL_TOPIC" == "/routed_goal_point" ]]; then
    python3 scripts/whitebox_stair_goal_router.py \
      --metadata "$SCENE_METADATA_PATH" &
    WHITEBOX_GOAL_ROUTER_PID=$!
  fi
  LAUNCH_ARGS=(
    "route_planner_config:=$ROUTE_PLANNER_CONFIG"
    "goal_topic:=$GOAL_TOPIC"
    "world_name:=$WORLD_NAME"
    "vehicleX:=$VEHICLE_X"
    "vehicleY:=$VEHICLE_Y"
    "vehicleTerrainMapTopic:=$VEHICLE_TERRAIN_MAP_TOPIC"
    "scene_mesh_path:=$SCENE_MESH_PATH"
    "scene_map_path:=$SCENE_MAP_PATH"
    "gazebo_gui:=$([[ "$NO_RVIZ" -eq 1 ]] && echo false || echo true)"
  )
  if [[ -n "$POSE_OVERRIDE_TOPIC" ]]; then
    LAUNCH_ARGS+=("poseOverrideTopic:=$POSE_OVERRIDE_TOPIC")
  fi
  ros2 launch vehicle_simulator system_simulation_with_route_planner_gazebo.launch.py "${LAUNCH_ARGS[@]}" &
  LAUNCH_PID=$!
elif [[ "$WHITEBOX_MODE" -eq 1 ]]; then
  python3 scripts/generate_whitebox_stair_test_scene.py --profile "$SCENE_PROFILE" >/dev/null
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
