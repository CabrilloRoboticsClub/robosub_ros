#!/usr/bin/env python3
"""
debug.py

Publish RPi4 Debug info to /debug_info
Also setup anything on the pi that needs to run upon start-up

Copyright (C) 2023-2024 Cabrillo Robotics Club

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
import rclpy 
from time import time
import serial
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class BMS(Node):
    """
    Reads system information from RPi4 and publishes it.
    Also setup anything on the pi that needs to run upon start-up.
    """
    def __init__(self):
        super().__init__("debug")
        # Setup node
        self._publisher = self.create_publisher(Float64MultiArray, "bms", 10)
        self.timer = self.create_timer(1.0, self.pub_callback)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.ser.reset_input_buffer() 
    
    def pub_callback(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * 9
        while True:
            if self.ser.in_waiting > 0:  # Check if there is data available to read
                try:
                    # Read a line, decode it from bytes to a string, and remove trailing whitespace
                    line = self.ser.readline().decode('utf-8').rstrip()
                    list = line.split(",")
                    if list[0] == "$DATA":
                        for i, val in enumerate(list[1:]):
                            msg.data[i] = float(val)
                        self._publisher.publish(msg)

                except UnicodeDecodeError:
                    # Handle potential decoding errors if non-UTF-8 data is received
                    print("Error decoding data.")


def main(args=None):
    rclpy.init(args=args)
    node = BMS()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()