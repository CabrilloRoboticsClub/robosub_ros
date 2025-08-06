#!/bin/bash
# jetson
# backout and fail if anything goes wrong
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(realpath "$SCRIPT_DIR/../../..")"

CONTAINER_NAME="isaac-ros-zed"

IMAGE_NAME="isaac-ros-zed-jetson:latest"

# runs the container with this script
docker run --rm -it \
    --runtime nvidia \
    --network host \
    --privileged \
    --ipc=host \
    --name "${CONTAINER_NAME}" \
    --env="DISPLAY=$DISPLAY" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}" \
    #--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="${WORKSPACE_DIR}:/workspaces/robosub" \
    --workdir="/workspaces/robosub" \
    --user "$(id -u):$(id -g)" \
    "${IMAGE_NAME}" \
    /bin/bash
    
    # on host machine run: xhost +local:docker
