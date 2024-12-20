
all: colcon-build

colcon-build:
	colcon build --symlink-install --allow-overriding sdformat_urdf

sim: colcon-build
	. ./install/setup.sh && ros2 launch lazershark_sim lazershark_sim.launch.py

clean:
	rm -rf build/ log/ install/
	
# sudo apt update
# sudo apt install ros-${ROS_DISTRO}-robot-localization
