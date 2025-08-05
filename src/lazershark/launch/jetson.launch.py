## will be launching zed camera launch file with jetson launch file
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    zed_wrapper_path = get_package_share_directory('zed_wrapper')

    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                zed_wrapper_path, 
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model': 'zed2i',
        }.items()
    )
        
        ## not needed for nvblox
    zed_depth_cloud = Node(
        package='zed_depth_cloud',
        executable='zed_depth_cloud',
        name='depth_cloud_node',
        output='screen', ## Debugging, show in terminal
        parameters=[{
            'input_topic': '/zed/zed_node/depth/depth_registered',
            'output_topic': '/lazer_shark/depth_cloud',
            'qos_depth': 10,
            'qos_history': 'KEEP_LAST',
            'qos_reliability': 'BEST_EFFORT',
            'qos_durability': 'VOLATILE'
        }],

    )

    zed_pose_odom = Node(
        package='zed_pose_odom_tracker',
        executable='zed_pose_odom_node',
        name='pose_odom_node',
        output='screen',
        parameters=[{
            'pose_topic': 'pose',
            'odom_topic': 'odom',
            'pose_child_frame': 'map_camera',
            'odom_child_frame': 'odom_camera'
        }],
    )

    depth_republisher = Node(
        package='depth_republisher',
        executable='depth_republisher',
        name='depth_republisher',
        output='screen',
        parameters=[{
            'input_depth_topic': '/zed/zed_node/depth/depth_registered',
            'input_info_topic':  '/zed/zed_node/depth/camera_info',
            'output_depth_topic': '/lazer_shark/depth/image',
            'output_info_topic':  '/lazer_shark/depth/camera_info'
        }],
    )


    nvblox_node = Node(
        package='isaac_ros_nvblox',
        executable='nvblox_node',
        name='nvblox',
        output='screen',
        parameters=[{  ## isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/specefications
            'input_qos': 'SENSOR_DATA',
            'use_tf_transforms': True, 
            'use_topic_transforms': False, ## must be set false to use tf transforms
            'use_color': False, ## color not needed for depth mapping, save3s recources

            'global_frame': 'map', ## odom, map used for global mapping 

            'voxel_size': 0.5,  ## meters
            'mapping_mode': 'static' ## options: static, dynamic, people
            ## https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/parameters.html
            
        }],
        remappings=[
            ('depth/image', '/lazer_shark/depth/image'),
            ('depth/camera_info', '/lazer_shark/depth/camera_info'),
        ]
    )

    return LaunchDescription([
            zed_camera,
            #zed_depth_cloud,
            zed_pose_odom,
            depth_republisher,
            nvblox_node
    ])