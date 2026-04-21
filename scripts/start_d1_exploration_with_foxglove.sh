#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BOUNDARY_FILE="${REPO_ROOT}/src/exploration_planner/tare_planner/data/boundary_from_odom.ply"
CONFIG_FILE="${REPO_ROOT}/src/exploration_planner/tare_planner/config/indoor_d1h_20x5_reference.yaml"
FOXGLOVE_SCRIPT="${SCRIPT_DIR}/start_foxglove_filtered.sh"
LOG_DIR="${REPO_ROOT}/.runtime_logs"
USE_FOXGLOVE=1
LOCAL_PLANNER_CHECK_ROT_OBSTACLE="false"
LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE="0.15"
LOCAL_PLANNER_GROUND_HEIGHT_THRE="0.05"
LOCAL_PLANNER_VEHICLE_LENGTH="0.6"
LOCAL_PLANNER_VEHICLE_WIDTH="0.6"
FOXGLOVE_DELAY_SECONDS="8"
EXPLORATION_READY_TIMEOUT_SECONDS="25"

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-42}"
ROS_LOCALHOST_ONLY_VALUE="${ROS_LOCALHOST_ONLY:-0}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [options]

Options:
  --boundary-file PATH    Boundary PLY file. Default:
                          ${BOUNDARY_FILE}
  --config-file PATH      TARE planner yaml file. Default:
                          ${CONFIG_FILE}
  --domain-id ID          ROS_DOMAIN_ID. Default: ${ROS_DOMAIN_ID_VALUE}
  --localhost-only 0|1    ROS_LOCALHOST_ONLY. Default: ${ROS_LOCALHOST_ONLY_VALUE}
  --no-foxglove           Start exploration only, skip foxglove_bridge
  --check-rot-obstacle 0|1
                          localPlanner checkRotObstacle. Default:
                          ${LOCAL_PLANNER_CHECK_ROT_OBSTACLE}
  --vehicle-length VALUE  localPlanner vehicleLength. Default:
                          ${LOCAL_PLANNER_VEHICLE_LENGTH}
  --vehicle-width VALUE   localPlanner vehicleWidth. Default:
                          ${LOCAL_PLANNER_VEHICLE_WIDTH}
  --obstacle-height-thre VALUE
                          localPlanner obstacleHeightThre. Default:
                          ${LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE}
  --ground-height-thre VALUE
                          localPlanner groundHeightThre. Default:
                          ${LOCAL_PLANNER_GROUND_HEIGHT_THRE}
  --foxglove-script PATH  Foxglove startup script. Default:
                          ${FOXGLOVE_SCRIPT}
  --foxglove-delay-seconds N
                          Wait N seconds after exploration launch before
                          starting foxglove_bridge. Default:
                          ${FOXGLOVE_DELAY_SECONDS}
  --ready-timeout-seconds N
                          Wait up to N seconds for key exploration topics
                          before starting foxglove_bridge. Default:
                          ${EXPLORATION_READY_TIMEOUT_SECONDS}
  --log-dir PATH          Directory for launch logs. Default:
                          ${LOG_DIR}
  --help                  Show this help

This script:
  1. kills stale upper-stack processes
  2. starts the D1 real-robot exploration launch
  3. optionally starts foxglove_bridge
  4. prints the log paths and Foxglove URL
EOF
}

resolve_path() {
  local raw="$1"
  if [[ "${raw}" = /* ]]; then
    printf '%s\n' "${raw}"
  else
    printf '%s\n' "${REPO_ROOT}/${raw}"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --boundary-file)
      BOUNDARY_FILE="$(resolve_path "$2")"
      shift 2
      ;;
    --config-file)
      CONFIG_FILE="$(resolve_path "$2")"
      shift 2
      ;;
    --domain-id)
      ROS_DOMAIN_ID_VALUE="$2"
      shift 2
      ;;
    --localhost-only)
      ROS_LOCALHOST_ONLY_VALUE="$2"
      shift 2
      ;;
    --no-foxglove)
      USE_FOXGLOVE=0
      shift
      ;;
    --check-rot-obstacle)
      LOCAL_PLANNER_CHECK_ROT_OBSTACLE="$2"
      shift 2
      ;;
    --vehicle-length)
      LOCAL_PLANNER_VEHICLE_LENGTH="$2"
      shift 2
      ;;
    --vehicle-width)
      LOCAL_PLANNER_VEHICLE_WIDTH="$2"
      shift 2
      ;;
    --obstacle-height-thre)
      LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE="$2"
      shift 2
      ;;
    --ground-height-thre)
      LOCAL_PLANNER_GROUND_HEIGHT_THRE="$2"
      shift 2
      ;;
    --foxglove-script)
      FOXGLOVE_SCRIPT="$(resolve_path "$2")"
      shift 2
      ;;
    --foxglove-delay-seconds)
      FOXGLOVE_DELAY_SECONDS="$2"
      shift 2
      ;;
    --ready-timeout-seconds)
      EXPLORATION_READY_TIMEOUT_SECONDS="$2"
      shift 2
      ;;
    --log-dir)
      LOG_DIR="$(resolve_path "$2")"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "${BOUNDARY_FILE}" ]]; then
  echo "Boundary file not found: ${BOUNDARY_FILE}" >&2
  exit 1
fi

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "Planner config file not found: ${CONFIG_FILE}" >&2
  exit 1
fi

if [[ "${USE_FOXGLOVE}" -eq 1 && ! -f "${FOXGLOVE_SCRIPT}" ]]; then
  echo "Foxglove startup script not found: ${FOXGLOVE_SCRIPT}" >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"

timestamp="$(date -u +%Y%m%d_%H%M%S)"
exploration_log="${LOG_DIR}/exploration_${timestamp}.log"
foxglove_log="${LOG_DIR}/foxglove_${timestamp}.log"

robot_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [[ -z "${robot_ip}" ]]; then
  robot_ip="10.1.1.36"
fi

echo "Cleaning previous upper-stack processes..."
"${SCRIPT_DIR}/cleanup_d1_nav_stack.sh"
sleep 1

echo "Starting exploration stack..."
nohup bash -lc "
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /opt/d1_ros2/local_setup.bash
  cd '${REPO_ROOT}'
  source ./source_workspace_setup.bash
  export D1_RUN_RVIZ=0
  export D1_ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
  export D1_ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY_VALUE}'
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY_VALUE}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION_VALUE}'
  ros2 launch vehicle_simulator system_real_robot_with_exploration_planner_d1.launch.py \
    d1_ros_domain_id:='${ROS_DOMAIN_ID_VALUE}' \
    d1_ros_localhost_only:='${ROS_LOCALHOST_ONLY_VALUE}' \
    exploration_boundary_file:='${BOUNDARY_FILE}' \
    exploration_planner_config_file:='${CONFIG_FILE}' \
    local_planner_vehicle_length:='${LOCAL_PLANNER_VEHICLE_LENGTH}' \
    local_planner_vehicle_width:='${LOCAL_PLANNER_VEHICLE_WIDTH}' \
    local_planner_check_rot_obstacle:='${LOCAL_PLANNER_CHECK_ROT_OBSTACLE}' \
    local_planner_obstacle_height_thre:='${LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE}' \
    local_planner_ground_height_thre:='${LOCAL_PLANNER_GROUND_HEIGHT_THRE}'
" >"${exploration_log}" 2>&1 &
exploration_pid=$!

sleep 2

foxglove_pid=""
if [[ "${USE_FOXGLOVE}" -eq 1 ]]; then
  echo "Waiting for exploration topics to settle before starting Foxglove..."
  ready_deadline=$((SECONDS + EXPLORATION_READY_TIMEOUT_SECONDS))
  while (( SECONDS < ready_deadline )); do
    if bash -lc "
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      source /opt/d1_ros2/local_setup.bash
      cd '${REPO_ROOT}'
      source ./source_workspace_setup.bash
      export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
      export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY_VALUE}'
      export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION_VALUE}'
      ros2 topic list | grep -q '^/registered_scan$'
      ros2 topic list | grep -q '^/state_estimation$'
    "; then
      break
    fi
    sleep 1
  done

  if [[ "${FOXGLOVE_DELAY_SECONDS}" != "0" ]]; then
    echo "Sleeping ${FOXGLOVE_DELAY_SECONDS}s before starting Foxglove bridge..."
    sleep "${FOXGLOVE_DELAY_SECONDS}"
  fi

  echo "Starting Foxglove bridge..."
  nohup bash -lc "
    set -eo pipefail
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
    export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY_VALUE}'
    export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION_VALUE}'
    '${FOXGLOVE_SCRIPT}'
  " >"${foxglove_log}" 2>&1 &
  foxglove_pid=$!
  sleep 1
fi

echo ""
echo "Started exploration launch"
echo "  pid: ${exploration_pid}"
echo "  boundary: ${BOUNDARY_FILE}"
echo "  config: ${CONFIG_FILE}"
echo "  local_planner.vehicleLength: ${LOCAL_PLANNER_VEHICLE_LENGTH}"
echo "  local_planner.vehicleWidth: ${LOCAL_PLANNER_VEHICLE_WIDTH}"
echo "  local_planner.checkRotObstacle: ${LOCAL_PLANNER_CHECK_ROT_OBSTACLE}"
echo "  local_planner.obstacleHeightThre: ${LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE}"
echo "  local_planner.groundHeightThre: ${LOCAL_PLANNER_GROUND_HEIGHT_THRE}"
echo "  log: ${exploration_log}"

if [[ "${USE_FOXGLOVE}" -eq 1 ]]; then
  echo ""
  echo "Started Foxglove bridge"
  echo "  pid: ${foxglove_pid}"
  echo "  log: ${foxglove_log}"
  echo "  url: ws://${robot_ip}:8765"
fi

echo ""
echo "Next:"
echo "  1. Wait a few seconds for ODIN and exploration to settle"
echo "  2. Connect Foxglove Desktop to ws://${robot_ip}:8765"
echo "  3. Check /registered_scan, /navigation_boundary, /way_point, /state_estimation"
