#!/usr/bin/env bash

set -eo pipefail

echo "Stopping ODIN mapping mode..."

patterns=(
  'start_odin_mapping_only.sh'
  '/opt/ros/humble/bin/ros2 run odin_ros_driver host_sdk_sample'
  '/home/robot/cmu_planner/install/odin_ros_driver/lib/odin_ros_driver/host_sdk_sample'
)

signal_round() {
  local sig="$1"
  for pattern in "${patterns[@]}"; do
    pkill "-${sig}" -f "${pattern}" || true
  done
}

signal_round INT
sleep 1
signal_round TERM
sleep 1
signal_round KILL
sleep 1

echo "Remaining mapping-related processes:"
ps -eo pid,cmd | rg 'start_odin_mapping_only|host_sdk_sample' || echo "None"
