# docker/x86_64/Dockerfile
# runs on x86_64 architecture for nvidia docker container (isaac_ros_common)
FROM nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_6f2a6bddf70fcd928f08e874635efe43

# install dependencies and zed sdk, install sudo now
RUN apt-get update && apt-get install -y \
    wget lsb-release gnupg sudo curl unzip \
    && rm -rf /var/lib/apt/lists/*

RUN wget -q --no-check-certificate https://download.stereolabs.com/zedsdk/5.0/cu12/ubuntu22?_gl=1*1c8ie5h*_gcl_au*MTE1MTE4MzM4MC4xNzUzNjcwOTU1 -O ZED_SDK.run && \
    chmod +x ZED_SDK.run && \
    ./ZED_SDK.run -- silent && \
    rm ZED_SDK.run

#setup environment variables
ENV ZED_SDK_ROOT="/usr/local/zed"
ENV CMAKE_PREFIX_PATH="${ZED_SDK_ROOT}:${CMAKE_PREFIX_PATH}"
ENV LD_LIBRARY_PATH="${ZED_SDK_ROOT}/lib:${LD_LIBRARY_PATH}"
ENV PATH="${ZED_SDK_ROOT}/tools:${PATH}"

# user setup, allowing for
ARG USERNAME=robosub-user
ARG USER_UID=1000
ARG USER_GID=1000

RUN if ! getent group $USER_GID >/dev/null; then \
        groupadd --gid $USER_GID $USERNAME; \
    fi && \
    if ! getent passwd $USER_UID >/dev/null; then \
        useradd --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

#user login
USER $USERNAME
WORKDIR /workspaces/robosub_ros



