## will be launching zed camera launch file with jetson launch file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='zed_depth_cloud',
            executable='zed_depth_cloud',
            name='depth_cloud_node',
            parameters=[{
                'input_topic': '/zed/zed_node/depth/depth_registered',
                'output_topic': '/lazer_shark/depth_cloud',
                'qos_depth': 10,
                'qos_history': 'KEEP_LAST',
                'qos_reliability': 'BEST_EFFORT',
                'qos_durability': 'VOLATILE'
            }],
            output='screen' ## Debugging, show in terminal
        ),

        Node(
            package='zed_pose_odom_tracker',
            executable='zed_pose_odom_node',
            name='pose_odom_node',
            parameters=[{
                'pose_topic': 'pose',
                'odom_topic': 'odom',
                'pose_child_frame': 'map_camera',
                'odom_child_frame': 'odom_camera'
            }],
            output='screen'
        )

    ])