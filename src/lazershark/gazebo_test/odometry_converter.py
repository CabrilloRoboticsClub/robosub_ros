import sys 

import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

class OdometryConverter(Node):
    def __init__(self):
        super().__init__("odometry_converter")

        self.subscription = self.create_subscription(Odometry, "sim_odometry", self.callback, 10)
        self.publisher = self.create_publisher(PoseWithCovariance, "sim_pose", 10)

    def callback(self, msg):
        pose_msg = PoseWithCovariance()
        pose_msg.data = msg.data.pose
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryConverter()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()    

if __name__ == "__main__":
    main(sys.argv)