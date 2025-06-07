from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown, OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """
    Generate a launch description for lazershark sim gazebo simulator.
    Launches all nodes requred for the sim.

    Launch with: `ros2 launch lazershark sim.launch.py`
    """
    # Get packages
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_lazershark = get_package_share_directory("lazershark")
    gz_sim_resource_path = os.path.join(pkg_lazershark, "sim")
    
    # Get robot sdf file
    sdf_file = os.path.join(
        pkg_lazershark, "sim", "robot", "lazershark_sim.sdf"
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

    # Bridge between ros and gazebo
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_lazershark, "sim", "config", "ros_gz_bridge.yaml"
                ),
            }
        ],
    )

    # Relay - use instead of transform when Gazebo is only publishing odom -> base_link
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=[
            "/gz/tf",
            "/tf",
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    # Gazebo server and gui
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_lazershark) / "sim" / "worlds" / "waterworld.sdf"}'
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )

    # RViz.
    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "iris.rviz"}'],
    #     condition=IfCondition(LaunchConfiguration("rviz")),
    # )
    
    # Kill gazebo on quit
    die_gazebo_die = RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: ExecuteProcess(cmd=["pkill " " -9 " " -f " "\"gz sim\""], shell=True)
        )
    )

    return LaunchDescription(
        [   
            SetParameter(name='use_sim_time', value=True),
            # Gazebo stuff
            DeclareLaunchArgument(
                "use_gz_tf", default_value="true", description="Use Gazebo TF."
            ),
            robot_state_publisher,
            bridge,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=bridge,
                    on_start=[
                        topic_tools_tf
                    ]
                )
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_sim_resource_path),
            # DeclareLaunchArgument(
            #     "rviz", default_value="true", description="Open RViz."
            # ),
            gz_sim_server,
            gz_sim_gui,
            die_gazebo_die,
            # rviz,

            # Lazershark nodes
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen'
            ),
            Node(
                package='lazershark',
                executable='pilot_input',
                name='pilot_input',
                output='screen'
            ),
            Node(
                package='lazershark',
                executable='thrust',
                name='thrust',
                output='screen'
            ),
            Node(
                package='lazershark',
                executable='imu_converter',
                name='imu_converter',
                output='screen'
            ),
        #     Node(
        #         package='robot_localization',
        #         executable='ekf_node',
        #         name='ekf_filter_node',
        #         output='screen',
        #         parameters=[os.path.join(get_package_share_directory("lazershark"), 'params', 'gazebo_ekf.yaml')],
        #    ),
        ]
    )
