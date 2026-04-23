#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BOUNDARY_FILE="${REPO_ROOT}/src/exploration_planner/tare_planner/data/boundary_from_odom.ply"
CONFIG_FILE="${REPO_ROOT}/src/exploration_planner/tare_planner/config/indoor_d1h_20x5_reference.yaml"
LOCAL_PLANNER_CHECK_ROT_OBSTACLE="false"
LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE="0.15"
LOCAL_PLANNER_GROUND_HEIGHT_THRE="0.05"
LOCAL_PLANNER_VEHICLE_LENGTH="0.6"
LOCAL_PLANNER_VEHICLE_WIDTH="0.6"

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-42}"
ROS_LOCALHOST_ONLY_VALUE="${ROS_LOCALHOST_ONLY:-0}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

MAP_DIR="${REPO_ROOT}/src/odin_ros_driver/map/manual_maps"
MAP_FILE="$(find "${MAP_DIR}" -maxdepth 1 -type f -name '*.bin' | sort | tail -1)"

ODIN_TEMPLATE_CONFIG_FILE="${REPO_ROOT}/src/odin_ros_driver/config/control_command_relocalization.yaml"
ODIN_DEFAULT_CONFIG_FILE="${REPO_ROOT}/src/odin_ros_driver/config/control_command.yaml"
ODIN_BACKUP_CONFIG_FILE="${REPO_ROOT}/src/odin_ros_driver/config/control_command.yaml.codex_backup"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [options]

Options:
  --boundary-file PATH    Boundary PLY file. Default:
                          ${BOUNDARY_FILE}
  --config-file PATH      TARE planner yaml file. Default:
                          ${CONFIG_FILE}
  --map-file PATH         Fixed ODIN relocalization map (.bin). Default:
                          latest file under ${MAP_DIR}
  --domain-id ID          ROS_DOMAIN_ID. Default: ${ROS_DOMAIN_ID_VALUE}
  --localhost-only 0|1    ROS_LOCALHOST_ONLY. Default: ${ROS_LOCALHOST_ONLY_VALUE}
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
  --help                  Show this help
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
    --map-file)
      MAP_FILE="$(resolve_path "$2")"
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

if [[ -z "${MAP_FILE}" || ! -f "${MAP_FILE}" ]]; then
  echo "Relocalization map file not found: ${MAP_FILE}" >&2
  exit 1
fi

cd "${REPO_ROOT}"

restore_config() {
  if [[ -f "${ODIN_BACKUP_CONFIG_FILE}" ]]; then
    mv -f "${ODIN_BACKUP_CONFIG_FILE}" "${ODIN_DEFAULT_CONFIG_FILE}"
  fi
}

trap restore_config EXIT

echo "Cleaning previous upper-stack processes..."
"${SCRIPT_DIR}/cleanup_d1_nav_stack.sh"
sleep 1

cp "${ODIN_DEFAULT_CONFIG_FILE}" "${ODIN_BACKUP_CONFIG_FILE}"
cp "${ODIN_TEMPLATE_CONFIG_FILE}" "${ODIN_DEFAULT_CONFIG_FILE}"
sed -i "s|__MAP_FILE__|${MAP_FILE}|g" "${ODIN_DEFAULT_CONFIG_FILE}"

echo "Starting exploration stack with ODIN relocalization in foreground..."
echo "  boundary: ${BOUNDARY_FILE}"
echo "  config: ${CONFIG_FILE}"
echo "  relocalization map: ${MAP_FILE}"
echo "  local_planner.vehicleLength: ${LOCAL_PLANNER_VEHICLE_LENGTH}"
echo "  local_planner.vehicleWidth: ${LOCAL_PLANNER_VEHICLE_WIDTH}"
echo "  local_planner.checkRotObstacle: ${LOCAL_PLANNER_CHECK_ROT_OBSTACLE}"
echo "  local_planner.obstacleHeightThre: ${LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE}"
echo "  local_planner.groundHeightThre: ${LOCAL_PLANNER_GROUND_HEIGHT_THRE}"
echo

set +u
source /opt/ros/humble/setup.bash
source /opt/d1_ros2/local_setup.bash
source ./source_workspace_setup.bash
set -u

export D1_RUN_RVIZ=0
export D1_ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export D1_ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY_VALUE}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY_VALUE}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_VALUE}"

exec ros2 launch vehicle_simulator system_real_robot_with_exploration_planner_d1.launch.py \
  d1_ros_domain_id:="${ROS_DOMAIN_ID_VALUE}" \
  d1_ros_localhost_only:="${ROS_LOCALHOST_ONLY_VALUE}" \
  exploration_boundary_file:="${BOUNDARY_FILE}" \
  exploration_planner_config_file:="${CONFIG_FILE}" \
  local_planner_vehicle_length:="${LOCAL_PLANNER_VEHICLE_LENGTH}" \
  local_planner_vehicle_width:="${LOCAL_PLANNER_VEHICLE_WIDTH}" \
  local_planner_check_rot_obstacle:="${LOCAL_PLANNER_CHECK_ROT_OBSTACLE}" \
  local_planner_obstacle_height_thre:="${LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE}" \
  local_planner_ground_height_thre:="${LOCAL_PLANNER_GROUND_HEIGHT_THRE}" \
  publish_static_map_to_odom_tf:=false
