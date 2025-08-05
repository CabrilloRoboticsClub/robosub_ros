#!/bin/bash

# backout and fail if anything goes wrong
set -e

SRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(realpath "$SCRIPT_DIR/../../..")"

CONTAINER_NAME="isaac-ros-zed"

IMAGE_NAME="isaac-ros-zed-x86_64:latest"

# runs the container with this script
docker run --rm -it \
    --gpus all \
    --net=host \
    --privileged \
    --name "${CONTAINER_NAME}" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${WORKSPACE_DIR}:/workspaces/robosub_ros" \
    --workdir="/workspaces/robosub" \
    "${IMAGE_NAME}" \
    /bin/bash
    