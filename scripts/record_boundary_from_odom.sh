#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ODIN_PID=""
STARTED_ODIN=0

cd "${REPO_ROOT}"

source /opt/ros/humble/setup.bash
source /opt/d1_ros2/local_setup.bash
source ./source_workspace_setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

ODIN_CONFIG_FILE="${REPO_ROOT}/src/odin_ros_driver/config/control_command.yaml"

cleanup() {
  if [[ "${STARTED_ODIN}" -eq 1 && -n "${ODIN_PID}" ]]; then
    echo
    echo "[record_boundary] stopping temporary ODIN driver (pid=${ODIN_PID})"
    kill -INT "${ODIN_PID}" 2>/dev/null || true
    wait "${ODIN_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT

if ros2 topic list 2>/dev/null | grep -qx '/odin1/odometry'; then
  echo "[record_boundary] detected existing /odin1/odometry, reusing current ODIN session"
else
  echo "[record_boundary] /odin1/odometry not found, starting ODIN driver..."
  ros2 run odin_ros_driver host_sdk_sample \
    --ros-args -p config_file:="${ODIN_CONFIG_FILE}" &
  ODIN_PID=$!
  STARTED_ODIN=1

  READY=0
  for _ in $(seq 1 40); do
    sleep 1
    if ros2 topic list 2>/dev/null | grep -qx '/odin1/odometry'; then
      READY=1
      break
    fi
  done

  if [[ "${READY}" -ne 1 ]]; then
    echo "[record_boundary] failed to detect /odin1/odometry after waiting 40s"
    exit 1
  fi

  echo "[record_boundary] ODIN odometry is ready"
fi

exec python3 "${SCRIPT_DIR}/record_boundary_from_odom.py" "$@"
