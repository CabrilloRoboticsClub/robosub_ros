import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped  ## gathering the stamped messages allows us to see when and what frame the data is in
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster 


class ZedPoseOdomTracker(Node):
    def __init__(self):
        super().__init__('zed_pose_odom_node')
        self.declare_parameter('pose_topic', 'pose') #/zed/zed_node/pose
        self.declare_parameter('odom_topic', 'odom') #/zed/zed_node/odom
        self.declare_parameter('pose_child_frame', 'map_camera')
        self.declare_parameter('odom_child_frame', 'odom_camera')

        self.pose_topic = self.get_parameter('pose_topic').value #.value will always give us the value current value of the parameter, even with overides done
        self.odom_topic = self.get_parameter('odom_topic').value
        self.pose_child_frame = self.get_parameter('pose_child_frame').value
        self.odom_child_frame = self.get_parameter('odom_child_frame').value

        # default qos zed values are already set but this allows better visability
        qos = QoSProfile( ## zed qos profiles: https://www.stereolabs.com/docs/ros2/zed-node#qos-profiles
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,  ## 'BEST_EFFORT is used for better performance but Reliable is used for compatibility' - https://www.stereolabs.com/docs/ros2/depth-sensing
            durability=DurabilityPolicy.VOLATILE
            
        ) 

        # subscribers
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"ZED Pose Odom Tracker Node started with pose topic: {self.pose_topic}, odom topic: {self.odom_topic}, pose child frame: {self.pose_child_frame}, odom child frame: {self.odom_child_frame}")

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # log the pose
        self.get_logger().info(f"[Pose] {msg.header.frame_id} => {self.pose_child_frame}: X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}")

        # Brodcast it as a tf2 transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = self.pose_child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Log the odometry pose
        self.get_logger().info(f"[Odom] {msg.header.frame_id} => {self.odom_child_frame}: X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}")

        # Broadcast it as a tf2 transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = self.odom_child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ZedPoseOdomTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
