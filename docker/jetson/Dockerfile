#docker/jetson/Dockerfile
# runs on NVIDIA Jetson architecture for robosub_ros
FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_77e6a678c2058abf96bedcb8f7dd4330

# install dependencies and zed sdk, install sudo now
RUN apt-get update && apt-get install -y \
    wget lsb-release gnupg sudo curl unzip \
    && rm -rf /var/lib/apt/lists/*
## grab proper zed sdk for jetson, depends on jetpack 6, 6.1/6.2: https://www.stereolabs.com/developers/release , 6.1 / 6.2 selected
RUN wget -q --no-check-certificate https://download.stereolabs.com/zedsdk/5.0/l4t36.4/jetsons?_gl=1*5rtv1o*_gcl_au*MTE1MTE4MzM4MC4xNzUzNjcwOTU1 -O ZED_SDK.run && \
    chmod +x ZED_SDK.run && \
    ./ZED_SDK.run --quiet --accept && \
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