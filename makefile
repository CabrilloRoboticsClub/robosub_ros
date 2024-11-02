
all: colcon-build 

colcon-build:
	colcon build --allow-overriding sdformat_urdf

sim: colcon-build
	. ./install/setup.sh && ros2 launch lazershark_sim lazershark_sim.launch.py

clean:
	rm -rf build/ log/ install/
	
