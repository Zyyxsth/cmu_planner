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

UNITY_MODE=${1:-docker}
TEST_ID=${2:-${TEST_ID:-0002}}
DOCKER_IMAGE=${DOCKER_IMAGE:-mtare-planner:latest}
RVIZ_CONFIG=${RVIZ_CONFIG:-src/mtare_planner/tare_planner/rviz/tare_planner_ground_robot1.rviz}
TARE_PREFIX=${TARE_PREFIX:-}

cleanup() {
    pkill -f "system_simulation_with_mtare_planner.launch.py" 2>/dev/null || true
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

    ./run_unity_instance_docker.sh stop 1 >/dev/null 2>&1 || true
}

trap cleanup EXIT

echo "============================================"
echo "MTARE Planner Robot1-Only Simulation"
echo "============================================"
echo "Mode: robot_1 in a 2-robot configuration"
echo "Unity mode: $UNITY_MODE"
echo "Test ID: $TEST_ID"
echo ""

echo "Cleaning up stale ROS/Unity processes..."
cleanup
sleep 2

pkill -9 "Model.x86_64" 2>/dev/null || true
sleep 1

if [ "$UNITY_MODE" = "docker" ]; then
    echo "Starting Unity container for robot_1..."
    ./run_unity_instance_docker.sh start 1 10001 "$DOCKER_IMAGE"
elif [ "$UNITY_MODE" = "host" ]; then
    echo "Host Unity startup is not automated."
    echo "Start a host Unity instance manually and point it to TCP 10001."
else
    echo "Skipping Unity startup. ROS stack only."
fi

sleep 3

echo "Starting ROS2 stack for robot_1 only..."
LAUNCH_ARGS=(
    mtare_planner_config:=indoor
    world_name:=unity
    use_boundary:=true
    robot_id:=1
    robot_num:=2
    robot_ns:=robot_1
    ros_tcp_port:=10001
    coordination:=true
    kAutoStart:=true
    test_id:="$TEST_ID"
    robot_types:=wheeled,wheeled
    vehicleX:=0.0
    vehicleY:=0.0
    vehicleYaw:=0.0
    launch_visualization:=true
    launch_joy:=false
)

if [ -n "$TARE_PREFIX" ]; then
    LAUNCH_ARGS+=("tare_prefix:=$TARE_PREFIX")
fi

ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch.py \
    "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

sleep 2

echo "Starting RViz..."
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!

echo ""
echo "============================================"
echo "System started"
echo "============================================"
echo "Active robot: /robot_1"
echo "Unity TCP target on host: 10001"
echo "Robot_0 is intentionally not started."
echo "Coordination mode follows test_id prefix (0 full comms, 1 relay, 2/3/5/6 rendezvous, 4 no comms)."
echo ""
echo "Close RViz or press Ctrl+C to stop."

wait "$RVIZ_PID" || true
