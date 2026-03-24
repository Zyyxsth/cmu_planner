#!/bin/bash
# Start multiple Unity instances for multi-robot simulation
# Each Unity instance connects to a different port (10000, 10001, ...)

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
UNITY_BASE_DIR="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity/environment"
ROBOT_NUM=${1:-2}

echo "Starting $ROBOT_NUM Unity instances..."

for ((i=0; i<ROBOT_NUM; i++)); do
    PORT=$((10000 + i))
    echo "Starting Unity instance $i on port $PORT..."
    
    # Unity may support command line arguments for ROS connection
    # Try different common formats
    "$UNITY_BASE_DIR/Model.x86_64" \
        -rosEndpointPort $PORT \
        -rosPort $PORT \
        --ros-endpoint-port $PORT \
        2>&1 | grep -v "rosEndpointPort\|rosPort\|--ros-endpoint-port" &
    
    sleep 5
done

echo "All Unity instances started."
echo "Ports: 10000-$((10000 + ROBOT_NUM - 1))"
