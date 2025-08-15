import os
import subprocess
import pathlib

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import FindExecutable

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
        - a50
        - bms
        - microros

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
    
    #region: NODES ON PI
    nodes = [   
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
        Node(
            package='lazershark',
            executable='bms',
            name='bms',
            output='screen',
            respawn=True,
            respawn_delay=0.0,
        ),
        Node(
            package='lazershark',
            executable='planner',
            name='planner',
            output='screen',
            respawn=True,
            respawn_delay=0,    
            parameters=[{
                "path":      "path.yaml",
                "default_z": 0.8,
            }]
        ),
        Node(
            package='lazershark',
            executable='thrust',
            name='thrust',
            output='screen',
            respawn=True,
            respawn_delay=0.0,
        ),
    ]
    #endregion: NODES ON PI

    #region: MICROROS
    microros_respawn_time = 0
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
                output="both",
                respawn=True,
                respawn_delay=microros_respawn_time
            ),
        )
        nodes.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=lambda event, ctx: subprocess.run("docker kill micro-ros-agent", shell=True)
                )
            ),
        )
    else: 
        print("No micro-ros serial device.")
    #endregion: MICROROS

    return LaunchDescription(nodes)
