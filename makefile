
all: colcon-build 

colcon-build:
	colcon build --allow-overriding sdformat_urdf
	
clean:
	rm -rf build/ log/ install/
