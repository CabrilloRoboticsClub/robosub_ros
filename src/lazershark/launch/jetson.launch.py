import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch all nodes and processes on the Jetson:

    Launch with: `ros2 launch lazershark jetson.launch.py`
    """
    explorer_camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="explorer_camera",
        namespace="bottom",
        output="screen",
        parameters=[{
            "video_device": "/dev/video0",  # camera inoput location
            "image_size": [640, 480],  # camera resolution
            "frame_rate": 30.0,  
            "pixel_format": "YUYV",
            "output_encoding": "bgr8",  # options: "bgr8", "rgb8", "yuyv"
            "camera_frame_id": "explorer_camera_frame",  # frame id for the camera
        }],
    )

    return LaunchDescription(
        [
            explorer_camera,
        ]
    )