#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

source ./source_workspace_setup.bash
export LD_LIBRARY_PATH="$SCRIPT_DIR/src/exploration_planner/tare_planner/or-tools/lib:${LD_LIBRARY_PATH:-}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

UNITY_MODE=${1:-docker}
DOCKER_IMAGE=${DOCKER_IMAGE:-mtare-planner:latest}

cleanup() {
    pkill -f "system_simulation_with_exploration_planner_ns.launch.py" 2>/dev/null || true
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
    rm -f /dev/shm/fastrtps_* /dev/shm/fastdds* 2>/dev/null || true

    ./run_unity_instance_docker.sh stop 1 >/dev/null 2>&1 || true
}

trap cleanup EXIT

echo "============================================"
echo "Single Exploration Planner Robot1-Only Simulation"
echo "============================================"
echo "Planner: exploration_planner (strict single-robot path)"
echo "ROS namespace: /robot_1"
echo "Unity mode: $UNITY_MODE"
echo ""

echo "Cleaning up stale ROS/Unity processes..."
cleanup
sleep 2

pkill -9 "Model.x86_64" 2>/dev/null || true
sleep 1

echo "Starting ROS2 stack for robot_1..."
ros2 launch vehicle_simulator system_simulation_with_exploration_planner_ns.launch.py \
    exploration_planner_config:=indoor \
    world_name:=unity \
    robot_id:=1 \
    robot_ns:=robot_1 \
    ros_tcp_port:=10001 \
    kAutoStart:=true \
    vehicleX:=0.0 \
    vehicleY:=0.0 \
    vehicleYaw:=0.0 \
    launch_visualization:=true \
    launch_joy:=false &

sleep 3

if [ "$UNITY_MODE" = "docker" ]; then
    echo "Starting Unity container for robot_1..."
    ./run_unity_instance_docker.sh start 1 10001 "$DOCKER_IMAGE"
elif [ "$UNITY_MODE" = "host" ]; then
    echo "Host Unity startup is not automated."
    echo "Start a host Unity instance manually and point it to TCP 10001."
else
    echo "Skipping Unity startup. ROS stack only."
fi

echo ""
echo "============================================"
echo "System started"
echo "============================================"
echo "Active robot: /robot_1"
echo "Planner path: strict single exploration_planner"
echo "Unity TCP target on host: 10001"
echo ""
echo "Open RViz manually if needed."

wait
