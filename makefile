
all: colcon-build 

colcon-build:
	colcon build 
	
clean:
	rm -rf build/ log/ install/
