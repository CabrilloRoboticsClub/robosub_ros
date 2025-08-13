import os
import subprocess
import pathlib

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import FindExecutable

def generate_launch_description():
    """
    Launch all nodes on the RPi for ROV
        - pilot_input
        - piloted_thrust
        - bms
        - microros: TODO!!!

    Launch with: `ros2 launch lazershark pi.launch.py`
    """
    # Get lazershark package
    pkg_lazershark = get_package_share_directory("lazershark")
    
    pilot_input = Node(
        package='lazershark',
        executable='pilot_input',
        name='pilot_input',
        output='screen',
        respawn=True,
        respawn_delay=0.0,
    )

    piloted_thrust = Node(
        package='lazershark',
        executable='piloted_thrust',
        name='piloted_thrust',
        output='screen',
        respawn=True,
        respawn_delay=0.0,
    )

    bms = Node(
        package='lazershark',
        executable='bms',
        name='bms',
        output='screen',
        respawn=True,
        respawn_delay=0.0,
    )

    return LaunchDescription([pilot_input, piloted_thrust, bms])
