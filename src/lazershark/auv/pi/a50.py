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

        self.declare_parameter("addr", "Not set.")
        self.declare_parameter("frameID", "Not set.")


def main(args=None):
    rclpy.init(args=args)
    node = A50()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
