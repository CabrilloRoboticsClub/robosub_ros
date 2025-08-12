import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch all nodes on the RPi:
        - robot_state_publisher
        - ekf_filter_node
        - naviguider0
        - naviguider1
        - targetpoint0
        - targetpoint1
        - a50

    Launch with: `ros2 launch lazershark pi.launch.py`
    """
    # Get lazershark package
    pkg_lazershark = get_package_share_directory("lazershark")

    # Get robot sdf file
    # TODO: Make a new SDF to reflect the correct frame locations.
    sdf_file = os.path.join(
        pkg_lazershark, "sim", "robot", "lazershark_sim.urdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    return LaunchDescription(
        [   
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[
                    {"robot_description": robot_desc},
                    {"frame_prefix": ""},
                ],
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[os.path.join(get_package_share_directory("lazershark"), 'params', 'auv_ekf.yaml')],
            ),
            Node(
                package='lazershark',
                executable='naviguider',
                name='naviguider0',
                output='screen',
                respawn=True,
                respawn_delay=0.0,
                parameters=[{
                    "dev": "/dev/ttyUSB1",
                    "frameID": "imu_naviguider0"
                }],
                remappings=[
                    ("imu/naviguider", "imu/naviguider0"),
                ]
            ),
            # Node(
            #     package='lazershark',
            #     executable='naviguider',
            #     name='naviguider1',
            #     output='screen',
            #     parameters=[{
            #         "dev":      "/dev/ttyUSB1",
            #         "frameID":  "imu_naviguider1"
            #     }],
            #     remappings=[
            #         ("imu/naviguider", "imu/naviguider1"),
            #     ]
            # ),
            # Node(
            #     package='lazershark',
            #     executable='targetpoint',
            #     name='targetpoint0',
            #     output='screen',
            #     respawn=True,
            #     respawn_delay=0.0,
            #     parameters=[{
            #         "dev":      "/dev/ttyUSB0",
            #         "frameID":  "imu_targetpoint0"
            #     }],
            #     remappings=[
            #         ("imu/targetpoint", "imu/targetpoint0"),
            #     ]
            # ),
            # Node(
            #     package='lazershark',
            #     executable='targetpoint',
            #     name='targetpoint1',
            #     output='screen',
            #     parameters=[{
            #         "dev":      "/dev/ttyUSB1",
            #         "frameID":  "imu_targetpoint1"
            #     }],
            #     remappings=[
            #         ("imu/targetpoint", "imu/targetpoint1"),
            #     ]
            # ),
            Node(
                package='lazershark',
                executable='a50',
                name='a50',
                output='screen',
                respawn=True,
                respawn_delay=0,    
                parameters=[{
                    "TCP_IP":   "10.42.0.98",
                    "TCP_PORT": "16171",
                    "frameID":  "dvl_a50"
                }]
            ),
        ]
    )
