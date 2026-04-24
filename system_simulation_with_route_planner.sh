#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./source_workspace_setup.bash

WHITEBOX_MODE=0
GAZEBO_MODE=0
NO_RVIZ=0

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
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--gazebo] [--whitebox] [--no-rviz]"
      exit 1
      ;;
  esac
done

cleanup() {
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

trap cleanup EXIT

if [[ "$GAZEBO_MODE" -eq 1 ]]; then
  python3 scripts/generate_whitebox_stair_test_scene.py >/dev/null
  ros2 launch vehicle_simulator system_simulation_with_route_planner_gazebo.launch.py \
    world_name:=whitebox_stair_test \
    gazebo_gui:=$([[ "$NO_RVIZ" -eq 1 ]] && echo false || echo true) &
  LAUNCH_PID=$!
elif [[ "$WHITEBOX_MODE" -eq 1 ]]; then
  python3 scripts/generate_whitebox_stair_test_scene.py >/dev/null
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
