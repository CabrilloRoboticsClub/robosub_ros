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
from time import sleep, monotonic_ns
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

sys.path.insert(0, '/workspaces/robosub_ros/lib')
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

        dev = self.get_parameter("dev").value
        self.frame_id = self.get_parameter("frameID").value

        self.publisher = self.create_publisher(Imu, "imu/targetpoint", 10)
        self.imu = targetpoint_lib.TargetPoint(dev=dev)
        self.imu.select_comp("kQuaternion", "kGyroX", "kGyroY", "kGyroZ", "kAccelX", "kAccelY", "kAccelZ")

        self.create_timer(1.0/30, self.callback)


    def callback(self):
        imu_msg = Imu()
        data = self.imu.read_data()

        imu_msg.header.frame_id = self.frame_id

        imu_msg.orientation.x = data["kQuaternion"][0]
        imu_msg.orientation.y = data["kQuaternion"][1]
        imu_msg.orientation.z = data["kQuaternion"][2]
        imu_msg.orientation.w = data["kQuaternion"][3]

        imu_msg.angular_velocity.x = data["kGyroX"]
        imu_msg.angular_velocity.y = data["kGyroY"]
        imu_msg.angular_velocity.z = data["kGyroZ"]

        imu_msg.linear_acceleration.x = data["kAccelX"]
        imu_msg.linear_acceleration.y = data["kAccelY"]
        imu_msg.linear_acceleration.z = data["kAccelZ"]

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
