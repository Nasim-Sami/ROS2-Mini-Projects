#!/usr/bin/env bash
# Runner script for Autonomous Drone Container
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="drone_jazzy_sim:latest"
CONTAINER_NAME="drone_jazzy_sim_instance"

# Allow local root/docker to access the host X11 display
xhost +local:root 2>/dev/null || true
xhost +local:docker 2>/dev/null || true

echo "=========================================================="
echo "🚁 Starting Drone Simulation Container: ${CONTAINER_NAME}"
echo "🎮 Hardware GPU Acceleration: /dev/dri enabled"
echo "📁 Workspace Mount: ${WORKSPACE_ROOT} -> /workspace"
echo "=========================================================="

if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
    echo "🔄 Resuming existing persistent container: ${CONTAINER_NAME}..."
    docker start -ai "${CONTAINER_NAME}"
else
    echo "🚀 Creating and launching persistent container: ${CONTAINER_NAME}..."
    docker run -it \
        --name "${CONTAINER_NAME}" \
        --net=host \
        --ipc=host \
        --device=/dev/dri \
        --env="DISPLAY=${DISPLAY}" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="${WORKSPACE_ROOT}:/workspace" \
        "${IMAGE_NAME}" \
        /bin/bash
fi
