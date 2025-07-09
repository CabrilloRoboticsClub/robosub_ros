import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

microros_serial_device = "/dev/ttyS0"
subprocess.run('sudo /usr/local/bin/openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program pico/seahawk.elf verify reset exit"',
                shell=True,
                check=True)

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
        pkg_lazershark, "lazershark_sim", "robot", "lazershark_sim.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    if microros_serial_device is not None and pathlib.Path(microros_serial_device).exists():
        nodes.append(
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='docker'),
                    " run " ,
                    " --rm ",
                    " --name micro-ros-agent ",
                    f" --device {microros_serial_device} ",
                    " --network host ",
                    " microros/micro-ros-agent:humble ",
                    f" serial --dev {microros_serial_device} baudrate=115200 ",
                ]],
                shell=True,
                name="micro-ros-agent",
                output='both',
                respawn=True,
                respawn_delay=respawn_time
            ),
        )

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
            Node(
                package='lazershark',
                executable='a50',
                name='a50',
                output='screen',
                parameters=[{
                    "TCP_IP":   "192.168.1.205",
                    "TCP_PORT": 16171,
                    "frameID":  "thruster_6"
                }]
            ),
        ]
    )
