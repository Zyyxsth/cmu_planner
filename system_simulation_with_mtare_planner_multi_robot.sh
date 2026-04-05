#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

source ./source_workspace_setup.bash
if [ -f ./install_mtare/setup.bash ]; then
    source ./install_mtare/setup.bash
fi
export LD_LIBRARY_PATH="$SCRIPT_DIR/src/mtare_planner/tare_planner/or-tools/lib:${LD_LIBRARY_PATH:-}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

ROBOT_NUM=${1:-2}
UNITY_MODE=${2:-docker}
TEST_ID=${3:-${TEST_ID:-0002}}
DOCKER_IMAGE=${DOCKER_IMAGE:-mtare-planner:latest}
RVIZ_CONFIG=${RVIZ_CONFIG:-src/mtare_planner/tare_planner/rviz/tare_planner_multi_robot.rviz}

cleanup() {
    pkill -f "system_simulation_with_mtare_planner_multi_robot.launch.py" 2>/dev/null || true
    pkill -f "tare_planner_node" 2>/dev/null || true
    pkill -f "vehicleSimulator" 2>/dev/null || true
    pkill -f "localPlanner" 2>/dev/null || true
    pkill -f "pathFollower" 2>/dev/null || true
    pkill -f "terrainAnalysis" 2>/dev/null || true
    pkill -f "terrainAnalysisExt" 2>/dev/null || true
    pkill -f "sensorScanGeneration" 2>/dev/null || true
    pkill -f "visualizationTools" 2>/dev/null || true
    pkill -f "default_server_endpoint" 2>/dev/null || true
    pkill -f "navigationBoundary" 2>/dev/null || true
    pkill -f "sim_image_repub" 2>/dev/null || true
    pkill -f "joy_node" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    rm -f /dev/shm/fastrtps_* /dev/shm/fastdds* 2>/dev/null || true

    for ((i=0; i<ROBOT_NUM; i++)); do
        ./run_unity_instance_docker.sh stop "$i" >/dev/null 2>&1 || true
    done
}

trap cleanup EXIT

echo "============================================"
echo "Multi-Robot Simulation with MTARE Planner"
echo "============================================"
echo "Robot number: $ROBOT_NUM"
echo "Unity mode: $UNITY_MODE"
echo "Test ID: $TEST_ID"
echo ""

echo "Cleaning up stale ROS/Unity processes..."
cleanup
sleep 2

pkill -9 "Model.x86_64" 2>/dev/null || true
sleep 1

if [ "$UNITY_MODE" = "docker" ]; then
    echo "Starting $ROBOT_NUM Unity instances in Docker..."
    for ((i=0; i<ROBOT_NUM; i++)); do
        ./run_unity_instance_docker.sh start "$i" "$((10000 + i))" "$DOCKER_IMAGE"
    done
elif [ "$UNITY_MODE" = "host" ]; then
    echo "Starting host Unity instances is not supported automatically."
    echo "Use docker mode, or start isolated Unity instances manually."
else
    echo "Skipping Unity startup. ROS stack only."
fi

sleep 3

echo "Starting ROS2 multi-robot stack..."
ros2 launch vehicle_simulator system_simulation_with_mtare_planner_multi_robot.launch.py \
    robot_num:="$ROBOT_NUM" \
    test_id:="$TEST_ID" \
    launch_joy:=false \
    launch_visualization:=true &
LAUNCH_PID=$!

sleep 2

echo "Starting RViz..."
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!

echo ""
echo "============================================"
echo "System started!"
echo "============================================"
echo ""
for ((i=0; i<ROBOT_NUM; i++)); do
    echo "Robot $i:"
    echo "  ROS namespace: /robot_$i"
    echo "  Unity TCP target on host: $((10000 + i))"
done
echo ""
echo "Coordination mode by test_id prefix:"
echo "  0xxx: full comms"
echo "  1xxx: relay comms"
echo "  2xxx: rendezvous tree"
echo "  3xxx: rendezvous middle"
echo "  4xxx: no comms"
echo "  5xxx: nearest rendezvous"
echo "  6xxx: farthest rendezvous"
echo ""
echo "Close RViz or press Ctrl+C to stop."

wait "$RVIZ_PID" || true
