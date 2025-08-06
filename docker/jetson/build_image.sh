#!/bin/bash
# run: chmod +x ./docker/jetson/build_image.sh first

# finds the scripts absolute directory and assigns to SCRIPT_DIR, essentially sets up the user to be able to run this script anywhere
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# place into workspace directory, moves 3 directories (kind of hard coded)
cd "$SCRIPT_DIR/../../.." || exit 1 # if fails to change directory, exit with error code 1

# build the docker image, user is current user, uid and gid are set to current users, -t tag name for image, -f specifies the Dockerfile location
docker build \
    --build-arg USERNAME=$USER \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    -t isaac-ros-zed-jetson:latest \
    -f robosub_ros/docker/jetson/Dockerfile \
    .