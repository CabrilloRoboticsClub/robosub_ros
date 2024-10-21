import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler

from launch.conditions import IfCondition

from launch.event_handlers import OnProcessStart

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for a iris quadcopter."""
    pkg_lazershark_sim = get_package_share_directory("lazershark_sim")

    # Robot description.

    # Ensure `SDF_PATH` is populated as `sdformat_urdf`` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Load SDF file.
    # TODO: Change to sdf
    sdf_file = os.path.join(
        pkg_lazershark_sim, "robot", "lazershark_sim.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()
        # print(robot_desc)

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

    # Bridge.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_lazershark_sim, "config", "ros_gz_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Transform - use if the model includes "gz::sim::systems::PosePublisher"
    #             and a filter is required.
    # topic_tools_tf = Node(
    #     package="topic_tools",
    #     executable="transform",
    #     arguments=[
    #         "/gz/tf",
    #         "/tf",
    #         "tf2_msgs/msg/TFMessage",
    #         "tf2_msgs.msg.TFMessage(transforms=[x for x in m.transforms if x.header.frame_id == 'odom'])",
    #         "--import",
    #         "tf2_msgs",
    #         "geometry_msgs",
    #     ],
    #     output="screen",
    #     respawn=True,
    # )

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

    return LaunchDescription(
        [
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
        ]
    )
