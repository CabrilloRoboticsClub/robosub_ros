"""
targetpoint.py

Copyright (C) 2022-2023 Cabrillo Robotics Club

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Cabrillo Robotics Club
6500 Soquel Drive Aptos, CA 95003
cabrillorobotics@gmail.com
"""
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from scipy.spatial.transform import Rotation as R
import numpy as np
import quaternion
from math import sqrt

sys.path.insert(0, '/home/liam/robosub_ros/lib')
import targetpoint_lib


class TargetPoint(Node):
    """
    Class which publishes Targetpoint data to the 
    """

    def __init__(self):
        """
        Initialize `targetpoint`
        """
        super().__init__("targetpoint")

        self.declare_parameter("dev", "Not set.")
        self.declare_parameter("frameID", "Not set.")
        self.G = 9.80665 # m/s^2

        dev = self.get_parameter("dev").value
        self.frame_id = self.get_parameter("frameID").value

        self.publisher = self.create_publisher(Imu, "imu/targetpoint", 10)
        self.imu = targetpoint_lib.TargetPoint(dev=dev)
        self.imu.select_comp("kQuaternion", "kGyroX", "kGyroY", "kGyroZ", "kAccelX", "kAccelY", "kAccelZ")

        self.create_timer(1.0/30, self.callback)


    def callback(self):
        imu_msg = Imu()
        data = self.imu.read_data()

        x = data["kQuaternion"][0]
        y = data["kQuaternion"][1]
        z = data["kQuaternion"][2]
        w = data["kQuaternion"][3]

        imu_msg.orientation.x = x
        imu_msg.orientation.y = y
        imu_msg.orientation.z = z
        imu_msg.orientation.w = w
        
        q = np.quaternion(sqrt(2)/2,sqrt(2)/2,0,0)  * np.quaternion(x,y,z,w)
        
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz',degrees=True)

        # self.get_logger().info(f"{rot}")
        # x = rot[0] + 180 if rot[0] < 0 else rot[0] - 180
        # y = -rot[1]
        # z = rot[2] + 180 if rot[2] < 0 else rot[2] - 180
        # self.get_logger().info(f"Roll: {} Pitch: {x} Yaw: {z}")

        imu_msg.angular_velocity.x = data["kGyroX"]
        imu_msg.angular_velocity.y = data["kGyroY"]
        imu_msg.angular_velocity.z = data["kGyroZ"]

        imu_msg.linear_acceleration.x = data["kAccelX"] * self.G
        imu_msg.linear_acceleration.y = data["kAccelY"] * self.G
        imu_msg.linear_acceleration.z = data["kAccelZ"] * self.G

        imu_msg.header.frame_id = self.frame_id
        imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()

        self.publisher.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
