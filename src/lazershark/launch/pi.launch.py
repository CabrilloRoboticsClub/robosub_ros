from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate a launch description for lazershark_sim gazebo simulator.
    Launch with: `ros2 launch lazershark_sim lazershark_sim.launch.py`
    """
    # Get packages
    pkg_lazershark = get_package_share_directory("lazershark")

    # Get robot sdf file
    sdf_file = os.path.join(
        pkg_lazershark, "lazershark_sim", "robot", "lazershark_sim.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    # Publish /tf and /tf_static.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ""},
        ],
    )

    return LaunchDescription(
        [   
            robot_state_publisher,
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[os.path.join(get_package_share_directory("lazershark"), 'params', 'ekf.yaml')],
            ),
            Node(
                package='lazershark',
                executable='naviguider',
                name='naviguider0',
                output='screen',
                parameters=[{
                    "dev": "/dev/ttyUSB0",
                    "frameID": "thruster_5" # TODO: Fix frames
                }],
                remappings=[
                    ("imu/naviguider", "imu/naviguider0"),
                ]
            ),
            Node(
                package='lazershark',
                executable='naviguider',
                name='naviguider1',
                output='screen',
                parameters=[{
                    "dev":      "/dev/ttyUSB1",
                    "frameID":  "thruster_6"
                }],
                remappings=[
                    ("imu/naviguider", "imu/naviguider1"),
                ]
            ),
            Node(
                package='lazershark',
                executable='targetpoint',
                name='targetpoint0',
                output='screen',
                parameters=[{
                    "dev":      "/dev/ttyUSB0",
                    "frameID":  "thruster_5"
                }],
                remappings=[
                    ("imu/targetpoint", "imu/targetpoint0"),
                ]
            ),
            Node(
                package='lazershark',
                executable='targetpoint',
                name='targetpoint1',
                output='screen',
                parameters=[{
                    "dev":      "/dev/ttyUSB1",
                    "frameID":  "thruster_6"
                }],
                remappings=[
                    ("imu/targetpoint", "imu/targetpoint1"),
                ]
            ),
        ]
    )
