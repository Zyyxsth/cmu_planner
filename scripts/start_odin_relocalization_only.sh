#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ODIN_TEMPLATE_CONFIG_FILE="${REPO_ROOT}/src/odin_ros_driver/config/control_command_relocalization.yaml"
ODIN_DEFAULT_CONFIG_FILE="${REPO_ROOT}/src/odin_ros_driver/config/control_command.yaml"
ODIN_BACKUP_CONFIG_FILE="${REPO_ROOT}/src/odin_ros_driver/config/control_command.yaml.codex_backup"
MAP_DIR="${REPO_ROOT}/src/odin_ros_driver/map/manual_maps"

MAP_FILE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map-file)
      MAP_FILE="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--map-file /abs/path/to/map.bin]"
      exit 1
      ;;
  esac
done

if [[ -z "${MAP_FILE}" ]]; then
  MAP_FILE="$(find "${MAP_DIR}" -maxdepth 1 -type f -name '*.bin' | sort | tail -1)"
fi

if [[ -z "${MAP_FILE}" || ! -f "${MAP_FILE}" ]]; then
  echo "No relocalization map found."
  echo "Expected a .bin under: ${MAP_DIR}"
  echo "Or provide one explicitly:"
  echo "  $0 --map-file /abs/path/to/map.bin"
  exit 1
fi

cd "${REPO_ROOT}"

source /opt/ros/humble/setup.bash
source /opt/d1_ros2/local_setup.bash
source ./source_workspace_setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

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
cp "${ODIN_TEMPLATE_CONFIG_FILE}" "${ODIN_DEFAULT_CONFIG_FILE}"
sed -i "s|__MAP_FILE__|${MAP_FILE}|g" "${ODIN_DEFAULT_CONFIG_FILE}"

echo "Starting ODIN relocalization mode in foreground..."
echo "  map: ${MAP_FILE}"
echo

ros2 run odin_ros_driver host_sdk_sample
