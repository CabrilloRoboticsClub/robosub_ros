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
        self.TCP_PORT = 16171 #int(self.get_parameter("TCP_PORT").value)
        self.frame_id = "dvl_a50" #self.get_parameter("frameID").value
        self.old_data = ""
        self.connect()

        self.msg = Odometry()

        self.timer = self.create_timer(1/30.0, self.callback)

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # self.socket.connect((self.TCP_IP, self.TCP_PORT))
            self.socket.connect(("10.42.0.98", 16171))
            self.socket.settimeout(1)
            self.socket.send(b'{"command":"set_config","parameters":{"range_mode":"0<=2"}}')
            self.get_logger().info(f"{self.get_data()}")
        except socket.error as e:
            self.get_logger().warn(f"Unable to connect to socket. Reconnecting.") 
            sleep(1)
            self.connect()

    def get_data(self):
        data = self.old_data
        
        while "\n" not in data:
            try: 
                new_data = self.socket.recv(1).decode()
            except socket.timeout as e:
                new_data = ""
                self.get_logger().warn(f"Socket timeout, reconnecting.\n{e}")
                self.connect()

            if len(new_data) == 0:
                self.get_logger().warn("Socket closed, reconnecting.")
                self.connect()
            else:
                data += new_data

        data, self.old_data = data.split("\n")

        return data

    def callback(self):
        self.msg.header.frame_id = "odom"
        self.msg.child_frame_id = self.frame_id
        self.msg.header.stamp.sec, self.msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()

        data = json.loads(self.get_data())
        # print(data)
        if data["type"] == "velocity":
            self.msg.pose.pose.position.z = float(data["altitude"])

            self.msg.twist.twist.linear.x =  float(data["vx"])
            self.msg.twist.twist.linear.y = -float(data["vy"])
            self.msg.twist.twist.linear.z = -float(data["vz"])
        elif data["type" ] == "position_local":
            self.msg.pose.pose.position.x =  float(data["x"])
            self.msg.pose.pose.position.y = -float(data["y"])
            self.msg.pose.pose.orientation.x = -float(data["yaw"])

        self.publisher.publish(self.msg)



def main(args=None):
    rclpy.init(args=args)
    node = A50()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
