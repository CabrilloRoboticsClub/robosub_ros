FROM docker.io/ubuntu:22.04

ARG USERNAME=student
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive

# Set the Timezone
ENV TZ="America/Los_Angeles"
RUN apt update -y && apt install -y tzdata \
    && echo "America/Los_Angeles" > /etc/timezone \
    && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime

# Un-minimize the system so we have manual pages, etc.
RUN yes | unminimize && \
    apt install -y man

# Install ROS2 Humble
RUN apt install -y software-properties-common \
    && add-apt-repository universe \
    && apt install curl -y \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64,arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update -y \
    && apt upgrade -y \
    && apt install -y ros-humble-desktop \
    && apt install -y ros-dev-tools

# Install Gazebo
RUN apt-get update -y \
    && apt-get install curl lsb-release gnupg \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64,arm64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update -y \
    && apt-get install -y gz-harmonic ros-humble-ros-gzharmonic

# Initialize rosdep 
RUN sudo rosdep init 

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -s /usr/bin/bash -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Extra packages as needed 
RUN apt install -y libgz-sim8-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    ros-humble-tinyxml2-vendor default-jre python3-pexpect python3-future mesa-utils strace \
    ros-humble-micro-ros-msgs ros-humble-robot-localization

# Install Python Libraries
# RUN apt install python3-pip
# RUN pip3 install simple_pid numpy

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

# Install Micro XRCE DDS Generator
# TODO: Remove ardupilot stuff
RUN cd /opt \
    && sudo git clone https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git \
    && sudo chown -R $USERNAME:$USERNAME Micro-XRCE-DDS-Gen \
    && cd Micro-XRCE-DDS-Gen \
    && ./gradlew assemble 

# Copy source for future tooling.
COPY --chown=$USERNAME:$USERNAME src/ /tmp/dep-repos/src 
WORKDIR /tmp/dep-repos 

# Install ArduPilot dependencies 
RUN export USER=$(whoami) \
    && ./src/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y 

# Install dependencies with rosdep 
ENV GZ_VERSION=harmonic
RUN . /opt/ros/humble/setup.sh && rosdep update && rosdep install --from-paths src --ignore-src -r -y

WORKDIR /home/${USERNAME}
#RUN sudo rm -rf /tmp/dep-repos 
COPY .devcontainer/default.bashrc /home/${USERNAME}/.bashrc
