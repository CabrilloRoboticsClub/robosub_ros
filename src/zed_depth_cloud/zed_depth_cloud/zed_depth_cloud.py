import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

class ZedDepthCloud(Node):
    def __init__(self):
        super().__init__('zed_depth_cloud')
        self.declare_parameter('input_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('output_topic', '/lazer_shark/depth_cloud')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_history', 'KEEP_LAST')
        self.declare_parameter('qos_reliability', 'BEST_EFFORT')
        self.declare_parameter('qos_durability', 'VOLATILE')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        qos_depth = self.get_parameter('qos_depth').value
        qos_history = self.get_parameter('qos_history').value
        qos_reliability = self.get_parameter('qos_reliability').value
        qos_durability = self.get_parameter('qos_durability').value


        qos = QoSProfile(
            depth=qos_depth,
            history=HistoryPolicy[qos_history],
            reliability=ReliabilityPolicy[qos_reliability],
            durability=DurabilityPolicy[qos_durability]
        )

        self.create_subscription(
            PointCloud2,
            input_topic,
            self.depth_callback,
            qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            output_topic,
            qos
        )
        self.get_logger().info(f"ZED Depth Cloud Node started with input topic: {inout_topic}, output topic: {output_topic}")

    def depth_callback(self, msg: PointCloud2):
        # proccessing can happen here or in the .yaml provided by zed: zed-ros2-wrapper/zed-wrapper/config/common_stereo.yaml
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZedDepthCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
