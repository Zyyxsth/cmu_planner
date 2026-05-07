#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR" || exit 1
source ./source_workspace_setup.bash

NO_RVIZ=0
SCENE_PROFILE="${WHITEBOX_SCENE_PROFILE:-office}"
EXPLORATION_CONFIG_FILE="$SCRIPT_DIR/src/exploration_planner/tare_planner/config/indoor_multilevel_gazebo.yaml"
POSE_OVERRIDE_TOPIC="/whitebox_vehicle_pose_override"
BOUNDARY_FILE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz)
      NO_RVIZ=1
      shift
      ;;
    --scene-profile)
      if [[ $# -lt 2 ]]; then
        echo "--scene-profile requires one value: office, realistic, or compact"
        exit 1
      fi
      SCENE_PROFILE="$2"
      shift 2
      ;;
    --config-file)
      if [[ $# -lt 2 ]]; then
        echo "--config-file requires one TARE yaml path"
        exit 1
      fi
      EXPLORATION_CONFIG_FILE="$2"
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
      echo "Usage: $0 [--no-rviz] [--scene-profile office|realistic|compact] [--config-file PATH] [--boundary-file PATH]"
      exit 1
      ;;
  esac
done

if [[ "$SCENE_PROFILE" != "office" && "$SCENE_PROFILE" != "realistic" && "$SCENE_PROFILE" != "compact" ]]; then
  echo "Invalid scene profile: $SCENE_PROFILE"
  echo "Expected: office, realistic, or compact"
  exit 1
fi

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

  kill_by_pattern "scripts/multilevel_stair_frontier_arbiter.py"
  kill_by_pattern "scripts/publish_whitebox_vehicle_terrain_map.py"
  kill_by_pattern "system_simulation_with_exploration_planner_gazebo.launch.py"
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

cleanup() {
  if [[ -n "${WHITEBOX_TERRAIN_PID:-}" ]]; then
    kill "$WHITEBOX_TERRAIN_PID" 2>/dev/null || true
  fi
  if [[ -n "${ARBITER_PID:-}" ]]; then
    kill "$ARBITER_PID" 2>/dev/null || true
  fi
  if [[ -n "${BOUNDARY_PID:-}" ]]; then
    kill "$BOUNDARY_PID" 2>/dev/null || true
  fi
  if [[ -n "${FLOOR2_TARE_PID:-}" ]]; then
    kill "$FLOOR2_TARE_PID" 2>/dev/null || true
  fi
  if [[ -n "${FLOOR1_TARE_PID:-}" ]]; then
    kill "$FLOOR1_TARE_PID" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
  fi
  cleanup_stale_sim_processes
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
  echo "[multilevel_explore] Waiting for nonzero /clock..."
  local deadline=$((SECONDS + 30))
  while [[ "$SECONDS" -lt "$deadline" ]]; do
    local sec
    sec=$(timeout 2s ros2 topic echo /clock --once --field clock.sec 2>/dev/null | tr -dc '0-9' || true)
    if [[ -n "$sec" && "$sec" -gt 0 ]]; then
      echo "[multilevel_explore] /clock is active."
      sleep 1
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for nonzero /clock."
  return 1
}

wait_for_topic_sample() {
  local topic="$1"
  local timeout_sec="${2:-30}"
  echo "[multilevel_explore] Waiting for $topic..."
  local deadline=$((SECONDS + timeout_sec))
  while [[ "$SECONDS" -lt "$deadline" ]]; do
    if timeout 3s ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      echo "[multilevel_explore] $topic is active."
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for one message on $topic."
  return 1
}

start_floor_tare() {
  local floor_ns="$1"
  local auto_start="$2"
  local waypoint_topic="/${floor_ns}/tare/way_point_raw"
  local finish_topic="/${floor_ns}/tare/exploration_finish_raw"
  local start_topic="/${floor_ns}/start_exploration"
  local runtime_topic="/${floor_ns}/runtime"
  local state_topic="/state_estimation_at_scan"
  local scan_topic="/registered_scan"
  local terrain_topic="/terrain_map"
  local terrain_ext_topic="/terrain_map_ext"

  if [[ "$floor_ns" == "floor2" ]]; then
    start_topic="/multilevel/start_floor2_tare"
    state_topic="/floor2/state_estimation_at_scan"
    scan_topic="/floor2/registered_scan"
    terrain_topic="/floor2/terrain_map"
    terrain_ext_topic="/floor2/terrain_map_ext"
  fi

  ros2 run tare_planner tare_planner_node --ros-args \
    -r __ns:=/"$floor_ns" \
    --params-file "$EXPLORATION_CONFIG_FILE" \
    -p use_sim_time:=true \
    -p kAutoStart:="$auto_start" \
    -p sub_start_exploration_topic_:="$start_topic" \
    -p sub_state_estimation_topic_:="$state_topic" \
    -p sub_registered_scan_topic_:="$scan_topic" \
    -p sub_terrain_map_topic_:="$terrain_topic" \
    -p sub_terrain_map_ext_topic_:="$terrain_ext_topic" \
    -p pub_waypoint_topic_:="$waypoint_topic" \
    -p pub_exploration_finish_topic_:="$finish_topic" \
    -p pub_runtime_topic_:="$runtime_topic"
}

wait_for_exploration_inputs() {
  wait_for_nonzero_clock || return 1
  wait_for_topic_sample /state_estimation 30 || return 1
  wait_for_topic_sample /lidar/points 30 || return 1
  wait_for_topic_sample /registered_scan 30 || return 1
  wait_for_topic_sample /state_estimation_at_scan 30 || return 1
  wait_for_topic_sample /terrain_map 30 || return 1
  wait_for_topic_sample /terrain_map_ext 30 || return 1
}

ros2 launch vehicle_simulator system_simulation_with_exploration_planner_gazebo.launch.py \
  exploration_planner_config:="indoor" \
  world_name:="$WORLD_NAME" \
  vehicleX:="$VEHICLE_X" \
  vehicleY:="$VEHICLE_Y" \
  vehicleTerrainMapTopic:="$VEHICLE_TERRAIN_MAP_TOPIC" \
  poseOverrideTopic:="$POSE_OVERRIDE_TOPIC" \
  scene_mesh_path:="$SCENE_MESH_PATH" \
  scene_map_path:="$SCENE_MAP_PATH" \
  boundary_file:="$BOUNDARY_FILE" \
  launch_tare:=false \
  gazebo_gui:=$([[ "$NO_RVIZ" -eq 1 ]] && echo false || echo true) &
LAUNCH_PID=$!

if ! wait_for_exploration_inputs; then
  echo "Exploration inputs did not become ready; not starting multilevel exploration."
  exit 1
fi

echo "[multilevel_explore] Starting navigation boundary before TARE auto-start."
ros2 run tare_planner navigationBoundary --ros-args \
  -p use_sim_time:=true \
  -p boundary_file_dir:="$BOUNDARY_FILE" \
  -p sendBoundary:=true \
  -p sendBoundaryInterval:=2 &
BOUNDARY_PID=$!

if ! wait_for_topic_sample /navigation_boundary 30; then
  echo "Navigation boundary did not become ready; not starting TARE."
  exit 1
fi

echo "[multilevel_explore] Starting known-stair frontier arbiter."
python3 scripts/multilevel_stair_frontier_arbiter.py \
  --metadata "$SCENE_METADATA_PATH" \
  --pose-override-topic "$POSE_OVERRIDE_TOPIC" &
ARBITER_PID=$!

wait_for_topic_sample /multilevel/stair_frontier_cloud 20 || echo "[multilevel_explore] Stair frontier visualization not seen yet."

echo "[multilevel_explore] Starting floor2 TARE in gated standby mode."
start_floor_tare "floor2" false &
FLOOR2_TARE_PID=$!

echo "[multilevel_explore] Starting floor1 TARE with raw waypoint output."
start_floor_tare "floor1" true &
FLOOR1_TARE_PID=$!

wait_for_topic_sample /floor1/tare/way_point_raw 30 || echo "[multilevel_explore] Floor1 raw TARE waypoint not seen yet."
wait_for_topic_sample /way_point 30 || echo "[multilevel_explore] Arbiter waypoint not seen yet."
wait_for_topic_sample /viewpoint_vis_cloud 30 || echo "[multilevel_explore] TARE viewpoint cloud not seen yet."
wait_for_topic_sample /selected_viewpoint_vis_cloud 30 || echo "[multilevel_explore] TARE selected viewpoint cloud not seen yet."

if [[ "$NO_RVIZ" -eq 1 ]]; then
  wait "$LAUNCH_PID"
else
  ros2 run rviz2 rviz2 -d src/exploration_planner/tare_planner/rviz/tare_planner_ground.rviz \
    --ros-args -p use_sim_time:=true
fi
