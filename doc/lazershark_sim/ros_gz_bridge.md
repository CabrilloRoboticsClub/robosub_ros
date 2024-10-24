# ROS Gazebo Bridge
See: [ros_gz_bridge](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge)

# Bridge communication between ROS and Gazebo
This package provides a network bridge which enables the exchange of messages
between ROS and Gazebo Transport.

The following message types can be bridged for topics:

| ROS type                                    | Gazebo type                                 |
|---------------------------------------------|:-------------------------------------------:|
| builtin_interfaces/msg/Time                 | ignition::msgs::Time                        |
| std_msgs/msg/Bool                           | ignition::msgs::Boolean                     |
| std_msgs/msg/ColorRGBA                      | ignition::msgs::Color                       |
| std_msgs/msg/Empty                          | ignition::msgs::Empty                       |
| std_msgs/msg/Float32                        | ignition::msgs::Float                       |
| std_msgs/msg/Float64                        | ignition::msgs::Double                      |
| std_msgs/msg/Header                         | ignition::msgs::Header                      |
| std_msgs/msg/Int32                          | ignition::msgs::Int32                       |
| std_msgs/msg/UInt32                         | ignition::msgs::UInt32                      |
| std_msgs/msg/String                         | ignition::msgs::StringMsg                   |
| geometry_msgs/msg/Wrench                    | ignition::msgs::Wrench                      |
| geometry_msgs/msg/WrenchStamped             | ignition::msgs::Wrench                      |
| geometry_msgs/msg/Quaternion                | ignition::msgs::Quaternion                  |
| geometry_msgs/msg/Vector3                   | ignition::msgs::Vector3d                    |
| geometry_msgs/msg/Point                     | ignition::msgs::Vector3d                    |
| geometry_msgs/msg/Pose                      | ignition::msgs::Pose                        |
| geometry_msgs/msg/PoseArray                 | ignition::msgs::Pose_V                      |
| geometry_msgs/msg/PoseWithCovariance        | ignition::msgs::PoseWithCovariance          |
| geometry_msgs/msg/PoseStamped               | ignition::msgs::Pose                        |
| geometry_msgs/msg/Transform                 | ignition::msgs::Pose                        |
| geometry_msgs/msg/TransformStamped          | ignition::msgs::Pose                        |
| geometry_msgs/msg/Twist                     | ignition::msgs::Twist                       |
| geometry_msgs/msg/TwistStamped              | ignition::msgs::Twist                       |
| geometry_msgs/msg/TwistWithCovariance       | ignition::msgs::TwistWithCovariance         |
| geometry_msgs/msg/TwistWithCovarianceStamped| ignition::msgs::TwistWithCovariance         |
| gps_msgs/GPSFix                             | ignition::msgs::NavSat                      |
| nav_msgs/msg/Odometry                       | ignition::msgs::Odometry                    |
| nav_msgs/msg/Odometry                       | ignition::msgs::OdometryWithCovariance      |
| rcl_interfaces/msg/ParameterValue           | ignition::msgs::Any                         |
| ros_gz_interfaces/msg/Altimeter             | ignition::msgs::Altimeter                   |
| ros_gz_interfaces/msg/Contact               | ignition::msgs::Contact                     |
| ros_gz_interfaces/msg/Contacts              | ignition::msgs::Contacts                    |
| ros_gz_interfaces/msg/Dataframe             | ignition::msgs::Dataframe                   |
| ros_gz_interfaces/msg/Entity                | ignition::msgs::Entity                      |
| ros_gz_interfaces/msg/EntityWrench          | ignition::msgs::EntityWrench                  |
| ros_gz_interfaces/msg/Float32Array          | ignition::msgs::Float_V                     |
| ros_gz_interfaces/msg/GuiCamera             | ignition::msgs::GUICamera                   |
| ros_gz_interfaces/msg/JointWrench           | ignition::msgs::JointWrench                 |
| ros_gz_interfaces/msg/Light                 | ignition::msgs::Light                       |
| ros_gz_interfaces/msg/SensorNoise           | ignition::msgs::SensorNoise                 |
| ros_gz_interfaces/msg/StringVec             | ignition::msgs::StringMsg_V                 |
| ros_gz_interfaces/msg/TrackVisual           | ignition::msgs::TrackVisual                 |
| ros_gz_interfaces/msg/VideoRecord           | ignition::msgs::VideoRecord                 |
| ros_gz_interfaces/msg/WorldControl          | ignition::msgs::WorldControl                |
| rosgraph_msgs/msg/Clock                     | ignition::msgs::Clock                       |
| sensor_msgs/msg/BatteryState                | ignition::msgs::BatteryState                |
| sensor_msgs/msg/CameraInfo                  | ignition::msgs::CameraInfo                  |
| sensor_msgs/msg/FluidPressure               | ignition::msgs::FluidPressure               |
| sensor_msgs/msg/Imu                         | ignition::msgs::IMU                         |
| sensor_msgs/msg/Image                       | ignition::msgs::Image                       |
| sensor_msgs/msg/JointState                  | ignition::msgs::Model                       |
| sensor_msgs/msg/Joy                         | ignition::msgs::Joy                         |
| sensor_msgs/msg/LaserScan                   | ignition::msgs::LaserScan                   |
| sensor_msgs/msg/MagneticField               | ignition::msgs::Magnetometer                |
| sensor_msgs/msg/NavSatFix                   | ignition::msgs::NavSat                      |
| sensor_msgs/msg/PointCloud2                 | ignition::msgs::PointCloudPacked            |
| tf2_msgs/msg/TFMessage                      | ignition::msgs::Pose_V                      |
| trajectory_msgs/msg/JointTrajectory         | ignition::msgs::JointTrajectory             |
| vision_msgs/msg/Detection3D                 | ignition::msgs::AnnotatedOriented3DBox      |
| vision_msgs/msg/Detection3DArray            | ignition::msgs::AnnotatedOriented3DBox_V    |

And the following for services:

| ROS type                             | Gazebo request             | Gazebo response       |
|--------------------------------------|:--------------------------:| --------------------- |
| ros_gz_interfaces/srv/ControlWorld   | ignition.msgs.WorldControl | ignition.msgs.Boolean |

Run `ros2 run ros_gz_bridge parameter_bridge -h` for instructions.


## Configuring the Bridge via YAML

When configuring many topics, it is easier to use a file-based configuration in a markup
language. In this case, the `ros_gz` bridge supports using a YAML file to configure the
various parameters.

The configuration file must be a YAML array of maps.
An example configuration for 5 bridges is below, showing the various ways that a
bridge may be specified:

```yaml
 # Set just topic name, applies to both
- topic_name: "chatter"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "ignition.msgs.StringMsg"

# Set just ROS topic name, applies to both
- ros_topic_name: "chatter_ros"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "ignition.msgs.StringMsg"

# Set just GZ topic name, applies to both
- gz_topic_name: "chatter_ign"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "ignition.msgs.StringMsg"

# Set each topic name explicitly
- ros_topic_name: "chatter_both_ros"
  gz_topic_name: "chatter_both_ign"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "ignition.msgs.StringMsg"

# Full set of configurations
- ros_topic_name: "ros_chatter"
  gz_topic_name: "ign_chatter"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "ignition.msgs.StringMsg"
  subscriber_queue: 5       # Default 10
  publisher_queue: 6        # Default 10
  lazy: true                # Default "false"
  direction: BIDIRECTIONAL  # Default "BIDIRECTIONAL" - Bridge both directions
                            # "GZ_TO_ROS" - Bridge Ignition topic to ROS
                            # "ROS_TO_GZ" - Bridge ROS topic to Ignition
```

To run the bridge node with the above configuration:
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$WORKSPACE/ros_gz/ros_gz_bridge/test/config/full.yaml
```

## YAML file from last year
```yaml
# Bridge of ROS and Gazebo topics
# See: https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge

# Bridge for each individual motor
- topic_name: "/motor_values/motor_0"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "ignition.msgs.Double"
  direction: ROS_TO_GZ

- topic_name: "/motor_values/motor_1"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "ignition.msgs.Double"
  direction: ROS_TO_GZ

- topic_name: "/motor_values/motor_2"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "ignition.msgs.Double"
  direction: ROS_TO_GZ

- topic_name: "/motor_values/motor_3"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "ignition.msgs.Double"
  direction: ROS_TO_GZ

- topic_name: "/motor_values/motor_4"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "ignition.msgs.Double"
  direction: ROS_TO_GZ

- topic_name: "/motor_values/motor_5"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "ignition.msgs.Double"
  direction: ROS_TO_GZ

- topic_name: "/motor_values/motor_6"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "ignition.msgs.Double"
  direction: ROS_TO_GZ

- topic_name: "/motor_values/motor_7"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "ignition.msgs.Double"
  direction: ROS_TO_GZ
```
