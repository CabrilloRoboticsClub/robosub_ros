# docker/x86_64/Dockerfile
# runs on x86_64 architecture for nvidia docker container (isaac_ros_common)

FROM nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_6f2a6bddf70fcd928f08e874635efe43

# install dependencies
RUN apt-get update && apt-get install -y \
    wget lsb-release gnupg sudo curl unzip \
    && rm -rf /var/lib/apt/lists/*