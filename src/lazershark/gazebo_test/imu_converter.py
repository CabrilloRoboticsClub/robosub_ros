import sys 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import random


class ImuConverter(Node):
    def __init__(self):
        super().__init__("imu_converter")

        self.subscription = self.create_subscription(Imu, "imu/sim", self.callback, 10)
        self.publisher = self.create_publisher(Imu, "imu/sim/corrected", 10)

    def callback(self, msg):
        corrected_msg = Imu()
        corrected_msg = msg
        msg.header.frame_id = "lazershark_sim/imu"
        msg.orientation_covariance          = [0.0] * 9
        msg.angular_velocity_covariance     = [0.0] * 9
        msg.linear_acceleration_covariance  = [0.0] * 9

        msg.orientation_covariance[0] = float(random.randrange(0, 1000))/1_000_000
        msg.orientation_covariance[4] = float(random.randrange(0, 1000))/1_000_000
        msg.orientation_covariance[8] = float(random.randrange(0, 1000))/1_000_000

        msg.angular_velocity_covariance[0] = float(random.randrange(0, 1000))/1_000_000
        msg.angular_velocity_covariance[4] = float(random.randrange(0, 1000))/1_000_000
        msg.angular_velocity_covariance[8] = float(random.randrange(0, 1000))/1_000_000

        msg.linear_acceleration_covariance[0] = float(random.randrange(0, 1000))/1_000_000
        msg.linear_acceleration_covariance[4] = float(random.randrange(0, 1000))/1_000_000
        msg.linear_acceleration_covariance[8] = float(random.randrange(0, 1000))/1_000_000

        self.publisher.publish(corrected_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuConverter()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()    

if __name__ == "__main__":
    main(sys.argv)