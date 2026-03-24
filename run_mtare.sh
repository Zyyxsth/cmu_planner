#!/bin/bash
# M-TARE Docker launcher script
# Usage: ./run_mtare.sh <environment> <comms_range> <robot_num> <robot_id> <network_interface>
#   - <environment>: The environment to explore (indoor, outdoor, etc.)
#   - <comms_range>: Communication range in meters
#   - <robot_num>: Total number of robots
#   - <robot_id>: Robot id, ranging from 0 to robot_num - 1
#   - <network_interface>: Name of the network interface (optional, for macvlan mode)
#
# Example:
#   Terminal 1: ./run_mtare.sh indoor 30 2 0
#   Terminal 2: ./run_mtare.sh indoor 30 2 1

set -e

# Parse arguments
ENVIRONMENT=${1:-indoor}
COMMS_RANGE=${2:-30}
ROBOT_NUM=${3:-2}
ROBOT_ID=${4:-0}
NETWORK_IFACE=${5:-}

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "ERROR: Docker is not installed"
    echo "Install: sudo apt install docker.io docker-compose"
    exit 1
fi

# Allow X11 forwarding
xhost +local:docker 2>/dev/null || xhost +

# Detect network interface if not provided
if [ -z "$NETWORK_IFACE" ]; then
    NETWORK_IFACE=$(ip route | grep default | awk '{print $5}' | head -1)
    echo "Auto-detected network interface: $NETWORK_IFACE"
fi

# Container name
CONTAINER_NAME="mtare_robot_${ROBOT_ID}"

# Check if container already exists
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    echo "Container $CONTAINER_NAME already exists, removing..."
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
fi

echo "======================================"
echo "Starting M-TARE Robot ${ROBOT_ID}"
echo "======================================"
echo "Environment:    $ENVIRONMENT"
echo "Comms Range:    ${COMMS_RANGE}m"
echo "Total Robots:   $ROBOT_NUM"
echo "Robot ID:       $ROBOT_ID"
echo "Network:        $NETWORK_IFACE"
echo "Container:      $CONTAINER_NAME"
echo ""

# Create tmux session name
TMUX_SESSION="mtare_robot_${ROBOT_ID}"

# Kill existing tmux session if exists
tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true

# Create new tmux session
tmux new-session -d -s "$TMUX_SESSION"

# Build docker image if not exists
if ! docker images | grep -q "mtare-planner"; then
    echo "Building Docker image (this may take a while)..."
    docker build -t mtare-planner:latest .
fi

# Start Docker container in tmux window
tmux send-keys -t "$TMUX_SESSION" "
docker run -it --rm \\
    --name $CONTAINER_NAME \\
    --hostname robot${ROBOT_ID} \\
    --network host \\
    -e DISPLAY=\$DISPLAY \\
    -e ROS_DOMAIN_ID=42 \\
    -e ROBOT_ID=$ROBOT_ID \\
    -e ROBOT_NUM=$ROBOT_NUM \\
    -e COMMS_RANGE=$COMMS_RANGE \\
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\
    -v $SCRIPT_DIR:/ros2_ws/src/host:ro \\
    --ipc=host \\
    mtare-planner:latest \\
    bash -c '
        source /opt/ros/humble/setup.bash
        source /ros2_ws/install/setup.bash
        export LD_LIBRARY_PATH=/ros2_ws/src/mtare_planner/tare_planner/or-tools/lib:\$LD_LIBRARY_PATH
        
        echo \"======================================\"
        echo \"Robot $ROBOT_ID of $ROBOT_NUM started\"
        echo \"======================================\"
        echo \"\"
        
        # Start Unity simulator
        echo \"Starting Unity simulator...\"
        /ros2_ws/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
        UNITY_PID=\$!
        sleep 5
        
        # Start M-TARE planner
        echo \"Starting M-TARE planner...\"
        ros2 launch vehicle_simulator system_simulation_with_mtare_planner.launch.py \\
            mtare_planner_config:=$ENVIRONMENT \\
            robot_id:=$ROBOT_ID \\
            kRobotNum:=$ROBOT_NUM \\
            kAutoStart:=true \\
            coordination:=true
        
        wait \$UNITY_PID
    '
" C-m

# Attach to tmux session
echo "Attaching to tmux session: $TMUX_SESSION"
echo ""
echo "To detach: Press Ctrl+B, then D"
echo "To reattach: tmux attach -t $TMUX_SESSION"
echo ""

sleep 1
tmux attach -t "$TMUX_SESSION"
