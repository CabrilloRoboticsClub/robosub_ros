import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch all nodes and processes on the Jetson:

    Launch with: `ros2 launch lazershark jetson.launch.py`
    """

    # bringup_dir = get_package_share_directory("nvblox_examples_bringup")

    # Launch Arguments
    # run_rviz_arg = DeclareLaunchArgument(
    #     "run_rviz", default_value="True",
    #     description="Whether to start RVIZ")

    # Get lazershark package
    pkg_lazershark = get_package_share_directory("lazershark")

    #region: NVBLOX
    # SEE: https://github.com/Tinker-Twins/NVIDIA-Isaac-ROS-Nvblox/blob/main/nvblox_examples/nvblox_examples_bringup/launch/nvblox/nvblox.launch.py

    # Conditionals for setup
    # setup_for_isaac_sim = IfCondition(
    #     LaunchConfiguration("setup_for_isaac_sim", default="False"))

    # Nvblox node
    nvblox = Node(
        name="nvblox_node",
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        parameters=[os.path.join(pkg_lazershark, "params", "nvblox.yaml")],
        remappings=[
            # TODO: Replace with real mappings
            ("/depth/image",       "TODO"),
            ("/depth/camera_info", "TODO"),
            ("/color/image",       "TODO"),
            ("/color/camera_info", "TODO"),
        ]
    )
    #endregion: NVBLOX
    
    #region: RVIZ
    #TODO: We might want to view voxels in rviz, but this can be added later
    # config_name = LaunchConfiguration("config_name", default='default.rviz')
    # config_path = PathJoinSubstitution([get_package_share_directory(
    #     'nvblox_examples_bringup'), 'config', 'rviz', config_name])
    # global_frame = LaunchConfiguration('global_frame', default='odom')

    # # Rviz node
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', config_path,       # set the config
    #                '-f', global_frame],     # overwrite the global frame
    #     output='screen')
    #endregion: RVIZ

    return LaunchDescription(
        [
            nvblox,
        ]
    )