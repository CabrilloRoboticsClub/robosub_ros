from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnShutdown
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description for a lazer_shark
    """
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_lazershark_sim = get_package_share_directory("lazershark_sim")

    # lazer_shark.
    lazer_shark = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("lazershark_sim"),
                        "launch",
                        "lazershark_auv.launch.py",
                    ]
                ),
            ]
        )
    )

    # Gazebo.
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_lazershark_sim) / "worlds" / "waterworld.sdf"}'
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

    die_gazebo_die = RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: ExecuteProcess(cmd=["pkill " " -9 " " -f " "\"gz sim\""], shell=True)
        )
    )

    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #     "rviz", default_value="true", description="Open RViz."
            # ),
            gz_sim_server,
            gz_sim_gui,
            lazer_shark,
            die_gazebo_die,
            # rviz,
        ]
    )
