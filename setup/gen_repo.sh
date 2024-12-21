#! /usr/bin/bash 

set -e 

# Setup humble
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Ardupilot repos
mkdir -p src 
vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
sudo apt update

# Setup ArduPilot dependencies 
export USER=$(whoami)
./src/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y 
echo "export PATH=$HOME/.local/bin:\$PATH" >> ~/.bashrc

# DDS gen for MicroROS
(
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
)

# Setup ardupilot gz
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
export GZ_VERSION=harmonic
echo 'export GZ_VERSION=harmonic' >> ~/.bashrc

# Rosdep to get dependencies. 
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
