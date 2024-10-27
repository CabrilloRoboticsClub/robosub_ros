# Resources 
- Example use of the graded bouyancy plugin: [graded_buoyancy.sdf](https://github.com/gazebosim/gz-sim/blob/gz-sim9/examples/worlds/graded_buoyancy.sdf)
- Various underwater plugins: [Simulating Autnomous Underwater Vehicles](https://gazebosim.org/api/gazebo/6/underwater_vehicles.html)
- Info on the hydrodynamics plugin [Hydrodynamics Class Reference](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1Hydrodynamics.html)
- Example simulation with a DVL: [dvl_world.sdf](https://github.com/gazebosim/gz-sim/blob/gz-sim9/examples/worlds/dvl_world.sdf)
- Simulating underwater robots example: [maritime_sim_tutorial](https://github.com/arjo129/maritime_sim_tutorial/) and [Simulating and Testing Underwater Robots in Gazebo](https://www.youtube.com/watch?v=JMevncnfM0Y)
- Information about the thruster plugin: [Thruster Class Reference](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1Thruster.html)
- MBARI model: [model.sdf.in](https://github.com/osrf/lrauv/blob/main/lrauv_description/models/tethys/model.sdf.in)


## Conneting ROS and GZ
```
ros2 topic pub /thruster_values/thruster_0 std_msgs/msg/Float64 'data: -31'

ros2 run ros_gz_bridge parameter_bridge thruster_values/thruster_0@std_msgs/msg/Float64@ignition.msgs.Double

gz topic -t /thruster_values/thruster_0 -e

ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$WORKSPACE/src/lazershark_sim/config/ros_gz_bridge.yaml
```

https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/src/bridge_config.cpp