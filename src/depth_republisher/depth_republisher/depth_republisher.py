#republisher for depth images and camera info, allowing specefied use for nvblox
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class DepthRepublisher(Node):
    def __init__(self):
        super().__init__('depth_republisher')
        # params for use in CLI or launch file
        self.declare_parameter('input_depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('input_info_topic', '/zed/zed_node/depth/camera_info')
        self.declare_parameter('output_depth_topic', '/lazer_shark/depth/image')
        self.declare_parameter('output_info_topic', '/lazer_shark/depth/camera_info')

        input_depth_topic = self.get_parameter('input_depth_topic').value
        input_info_topic = self.get_parameter('input_info_topic').value
        output_depth_topic = self.get_parameter('output_depth_topic').value
        output_info_topic = self.get_parameter('output_info_topic').value

        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.depth_subscriber = self.create_subscription(
            Image,
            input_depth_topic,
            self.depth_callback,
            qos_profile
        )

        self.info_subscriber = self.create_subscription(
            CameraInfo,
            input_info_topic,
            self.info_callback,
            qos_profile
        )

        self.depth_publisher = self.create_publisher(
            Image,
            output_depth_topic,
            qos_profile
        )

        self.info_publisher = self.create_publisher(
            CameraInfo,
            output_info_topic,
            qos_profile
        )

    def depth_callback(self, msg):
        self.depth_publisher.publish(msg)
    
    def info_callback(self, msg):
        self.info_publisher.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = DepthRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
