#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
LAUNCH_PID=""

cleanup() {
  trap - EXIT INT TERM
  cd "$SCRIPT_DIR"
  ./scripts/cleanup_d1_nav_stack.sh
}

should_run_rviz() {
  local mode="${D1_RUN_RVIZ:-auto}"
  case "$mode" in
    1|true|TRUE|yes|YES)
      return 0
      ;;
    0|false|FALSE|no|NO)
      return 1
      ;;
    auto)
      [[ -n "${DISPLAY:-}" ]]
      ;;
    *)
      return 1
      ;;
  esac
}

trap cleanup EXIT INT TERM

cd "$SCRIPT_DIR"
./scripts/cleanup_d1_nav_stack.sh
source ./source_workspace_setup.bash

ros2 launch vehicle_simulator system_real_robot_with_route_planner_d1.launch.py \
  d1_ros_domain_id:="${D1_ROS_DOMAIN_ID:-42}" \
  d1_ros_localhost_only:="${D1_ROS_LOCALHOST_ONLY:-1}" &
LAUNCH_PID=$!
sleep 1

if should_run_rviz; then
  ros2 run rviz2 rviz2 -d src/route_planner/far_planner/rviz/default.rviz
else
  wait "$LAUNCH_PID"
fi
