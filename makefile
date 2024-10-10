
all: colcon-build 

colcon-build:
	colcon build --allow-overriding sdformat_urdf
	
clean:
	rm -rf build/ log/ install/
	rm -rf src/ardupilot/build src/ardupilot/.lock* src/ardupilot/tasklist.json
	rm -rf $$(find src/ardupilot -name __pycache__ -type d) 
	
