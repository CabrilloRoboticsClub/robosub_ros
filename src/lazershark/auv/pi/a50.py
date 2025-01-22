"""
a50.py

Copyright (C) 2024-2025 Cabrillo Robotics Club

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
from time import sleep
import socket
import json

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class A50(Node):
    """
    Class which publishes Waterlinked DVL a50 data to topic `dvl/a50`.
    """
    
    def __init__(self):
        """
        Initialize `a50` node.
        """
        super().__init__("a50")

        self.publisher = self.create_publisher(Odometry, "dvl/a50", 10)

        self.declare_parameter("TCP_IP",    "Not set.")
        self.declare_parameter("TCP_PORT",  "Not set.")
        self.declare_parameter("frameID",   "Not set.")

        self.TCP_IP   = self.get_parameter("TCP_IP").value
        self.TCP_PORT = self.get_parameter("TCP_PORT").value
        self.frame_id = self.get_parameter("frameID").value
        self.old_data = ""

        self.connect()

        self.timer = self.create_timer(1/10.0, self.callback)

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.TCP_IP, self.TCP_PORT))
            self.socket.settimeout(1)
        except socket.error as e:
            self.get_logger().warn(f"Unable to connect to socket. Reconnecting.") 
            sleep(1)
            self.connect()



def main(args=None):
    rclpy.init(args=args)
    node = A50()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
