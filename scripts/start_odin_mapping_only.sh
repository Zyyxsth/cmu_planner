#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ODIN_CONFIG_FILE="${REPO_ROOT}/src/odin_ros_driver/config/control_command_mapping.yaml"
ODIN_DEFAULT_CONFIG_FILE="${REPO_ROOT}/src/odin_ros_driver/config/control_command.yaml"
ODIN_BACKUP_CONFIG_FILE="${REPO_ROOT}/src/odin_ros_driver/config/control_command.yaml.codex_backup"

cd "${REPO_ROOT}"

source /opt/ros/humble/setup.bash
source /opt/d1_ros2/local_setup.bash
source ./source_workspace_setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "${REPO_ROOT}/src/odin_ros_driver/map/manual_maps"

restore_config() {
  if [[ -f "${ODIN_BACKUP_CONFIG_FILE}" ]]; then
    mv -f "${ODIN_BACKUP_CONFIG_FILE}" "${ODIN_DEFAULT_CONFIG_FILE}"
  fi
}

trap restore_config EXIT

echo "Cleaning previous upper-stack processes..."
"${REPO_ROOT}/scripts/cleanup_d1_nav_stack.sh"
pkill -f foxglove_bridge || true

cp "${ODIN_DEFAULT_CONFIG_FILE}" "${ODIN_BACKUP_CONFIG_FILE}"
cp "${ODIN_CONFIG_FILE}" "${ODIN_DEFAULT_CONFIG_FILE}"

echo "Starting ODIN mapping mode in foreground..."
echo "  config: ${ODIN_CONFIG_FILE}"
echo "  map output dir: ${REPO_ROOT}/src/odin_ros_driver/map/manual_maps"
echo
echo "When finished scanning, save the map with:"
echo "  ./scripts/save_odin_map.sh"
echo

ros2 run odin_ros_driver host_sdk_sample
