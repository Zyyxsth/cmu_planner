#!/bin/bash

set -euo pipefail

ACTION=${1:-start}
ROBOT_ID=${2:-0}
HOST_TCP_PORT=${3:-$((10000 + ROBOT_ID))}
DOCKER_IMAGE=${4:-mtare-planner:latest}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
UNITY_BIN="/workspace/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
CONTAINER_NAME="mtare_unity_${ROBOT_ID}"

docker_cmd() {
    if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
        docker "$@"
    elif command -v sudo >/dev/null 2>&1; then
        sudo docker "$@"
    else
        echo "Docker is required but not available." >&2
        exit 1
    fi
}

if [ "$ACTION" = "stop" ]; then
    docker_cmd rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
    exit 0
fi

if [ "$ACTION" != "start" ]; then
    echo "Usage: $0 <start|stop> [robot_id] [host_tcp_port] [docker_image]" >&2
    exit 1
fi

if [ ! -x "$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64" ]; then
    echo "Unity executable not found at $SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64" >&2
    exit 1
fi

if command -v xhost >/dev/null 2>&1; then
    xhost +SI:localuser:root >/dev/null 2>&1 || true
fi

if ! docker_cmd run --rm --entrypoint bash "$DOCKER_IMAGE" -lc 'command -v socat >/dev/null 2>&1'; then
    echo "Docker image $DOCKER_IMAGE is missing socat. Rebuild it with: docker build -t $DOCKER_IMAGE ." >&2
    exit 1
fi

docker_cmd rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

echo "Starting Unity container $CONTAINER_NAME -> host TCP $HOST_TCP_PORT"
DOCKER_ARGS=(
    run -d --rm
    --name "$CONTAINER_NAME"
    --add-host=host.docker.internal:host-gateway
    -e DISPLAY="${DISPLAY:-:0}"
    -e QT_X11_NO_MITSHM=1
    -e HOST_TCP_PORT="$HOST_TCP_PORT"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "$SCRIPT_DIR":/workspace:rw
)

if command -v nvidia-smi >/dev/null 2>&1; then
    if docker_cmd run --rm --runtime=nvidia --gpus all --entrypoint bash "$DOCKER_IMAGE" -lc 'true' >/dev/null 2>&1; then
        DOCKER_ARGS+=(
            --runtime nvidia
            --gpus all
            -e NVIDIA_VISIBLE_DEVICES=all
            -e NVIDIA_DRIVER_CAPABILITIES=all
        )
    else
        echo "NVIDIA GPU detected on host, but Docker GPU passthrough is not configured." >&2
        echo "Unity is currently falling back to llvmpipe software rendering in Docker, which is too slow for usable sensor output." >&2
        echo "Install and configure nvidia-container-toolkit, or run Unity on the host instead of Docker." >&2
        exit 1
    fi
elif [ -e /dev/dri ]; then
    DOCKER_ARGS+=(--device /dev/dri:/dev/dri)
fi

DOCKER_ARGS+=(
    "$DOCKER_IMAGE"
    bash -lc "
        set -euo pipefail
        socat TCP-LISTEN:10000,bind=127.0.0.1,reuseaddr,fork TCP:host.docker.internal:\${HOST_TCP_PORT} &
        exec $UNITY_BIN
    "
)

docker_cmd "${DOCKER_ARGS[@]}" >/dev/null
