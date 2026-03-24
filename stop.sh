#!/bin/bash
# Stop M-TARE robots
# Usage:
#   ./stop.sh          # Stop all robots
#   ./stop.sh <id>     # Stop specific robot with id

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

if [ $# -eq 0 ]; then
    # Stop all robots
    echo "Stopping all M-TARE robots..."
    
    # Kill all mtare tmux sessions
    tmux ls 2>/dev/null | grep "mtare_robot_" | cut -d: -f1 | while read session; do
        echo "Killing tmux session: $session"
        tmux kill-session -t "$session" 2>/dev/null || true
    done
    
    # Stop all mtare docker containers
    docker ps -q --filter "name=mtare_robot_" | while read container; do
        echo "Stopping container: $container"
        docker stop "$container" 2>/dev/null || true
    done
    
    # Remove all mtare docker containers
    docker ps -aq --filter "name=mtare_robot_" | while read container; do
        docker rm "$container" 2>/dev/null || true
    done
    
    echo "All robots stopped."
    
else
    # Stop specific robot
    ROBOT_ID=$1
    CONTAINER_NAME="mtare_robot_${ROBOT_ID}"
    TMUX_SESSION="mtare_robot_${ROBOT_ID}"
    
    echo "Stopping robot $ROBOT_ID..."
    
    # Kill tmux session
    tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || echo "Tmux session $TMUX_SESSION not found"
    
    # Stop and remove container
    docker stop "$CONTAINER_NAME" 2>/dev/null || echo "Container $CONTAINER_NAME not running"
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
    
    echo "Robot $ROBOT_ID stopped."
fi

# Revoke X11 access
xhost -local:docker 2>/dev/null || true
