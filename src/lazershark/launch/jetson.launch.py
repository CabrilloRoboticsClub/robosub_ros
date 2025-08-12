import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Launch all nodes and processes on the Jetson:

    Launch with: `ros2 launch lazershark jetson.launch.py`
    """

    zed_od_params = os.path.join(
        get_package_share_directory('lazershark'),
        'params',
        'robosub_od.yaml'
    )

    explorer_camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="explorer_camera",
        namespace="bottom_explorer_camera_frame", # just in case there is conflicting params for zed
        output="screen",
        parameters=[{
            "video_device": "/dev/video0",  # camera input location
            "image_size": [640, 480],  # camera resolution
            "frame_rate": 30.0,  
            "pixel_format": "YUYV",
            "output_encoding": "bgr8",  # options: "bgr8", "rgb8", "yuyv"
            "camera_frame_id": "explorer_camera_frame",  # frame id for the camera
        }],
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zed_wrapper'),
                        'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            "camera_model": "zed2i",
            "camera_name":  "zed2i",
            "object_detection_config_path": zed_od_params,
            "custom_object_detection_config_path": zed_od_params,
        }.items()
    )


    yolov5_node = Node(
        package="lazershark",
        executable="yolov5_node",
        name="yolov5_node",
        output="screen",
        parameters=[{
            "yolo_dir": "/home/ubuntu/yolov5", ## need to update this
            "weights": "/home/ubuntu/yolov5/best.pt", ## need to update this
            "conf_thres": 0.30,
            "device": "auto",
            "image_topic": "/bottom/image_raw",
            "output_topic": "/bottom/detections",
            "use_class_names": True
        }]
    )

    return LaunchDescription(
        [
            explorer_camera,
            zed_launch,
            yolov5_node,
        ]
    )