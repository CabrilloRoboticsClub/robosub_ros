
all: colcon-build

colcon-build:
	colcon build --symlink-install --allow-overriding sdformat_urdf

sim: colcon-build
	. ./install/setup.sh && ros2 launch lazershark lazershark_sim.launch.py

clean:
	rm -rf build/ log/ install/
	
