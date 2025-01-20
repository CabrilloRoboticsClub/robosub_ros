"""
naviguider.py

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
from sensor_msgs.msg import Imu

from serial import Serial
sys.path.insert(0, '/workspaces/robosub_ros/lib')
from naviguider_simpleserial.src.naviguider_simpleserial import (
    decode_line,
    encode_set_rotation_vector_sensor_rate,
    encode_set_linear_acceleration_sensor_rate,
    encode_system_restart,
    encode_set_gyroscope_sensor_rate,
    sensor_event,
)


class NaviGuider(Node):
    """
    Class which publishes Naviguider data to topic `imu/naviguider`.
    """
    
    def __init__(self):
        """
        Initialize `naviguider` node.
        """
        super().__init__("naviguider")

        self.declare_parameter("dev", "Not set.")
        self.declare_parameter("frameID", "Not set.")
        self.SETUP_DELAY = 2
    
        self.publisher = self.create_publisher(Imu, "imu/naviguider", 10) 
        self.serial_init()
        self.get_data()

    def serial_init(self):
        dev = self.get_parameter("dev").value
        self.serial_port = Serial(dev, 115200)
        self.get_logger().info(f"Port {dev} open: {self.serial_port.is_open}")
        
        self.serial_port.write(encode_system_restart().encode())
        sleep(self.SETUP_DELAY)
        self.serial_port.write(encode_set_rotation_vector_sensor_rate(15).encode())
        sleep(self.SETUP_DELAY)
        self.serial_port.write(encode_set_linear_acceleration_sensor_rate(15).encode())
        sleep(self.SETUP_DELAY)
        self.serial_port.write(encode_set_gyroscope_sensor_rate(15).encode())
        sleep(self.SETUP_DELAY)

        self.get_logger().info(f"Serial init complete.")

    def get_data(self):
        imu_msg = Imu()
        imu_msg.header.frame_id = self.get_parameter("frameID").value
        update_map = 0b000

        while True:
            try:
                data = self.serial_port.readline().decode("utf-8").strip()
                if data:
                    event = decode_line(data)
            except ValueError as e:
                self.get_logger().warn(f"ValueError: {e}")
                continue
            except TypeError:
                self.get_logger().warn(f"TypeError: {e}")
                continue
            if event:
                if type(event) is sensor_event.RotationVectorSensorEvent:
                    imu_msg.orientation.x = event.qx
                    imu_msg.orientation.y = event.qy
                    imu_msg.orientation.z = event.qz
                    imu_msg.orientation.w = event.qw
                    update_map |= 0b001
                if type(event) is sensor_event.GyroscopeSensorEvent:
                    imu_msg.angular_velocity.x = event.x
                    imu_msg.angular_velocity.y = event.y
                    imu_msg.angular_velocity.z = event.z
                    update_map |= 0b010
                if type(event) is sensor_event.LinearAccelerationSensorEvent:
                    imu_msg.linear_acceleration.x = event.x
                    imu_msg.linear_acceleration.y = event.y
                    imu_msg.linear_acceleration.z = event.z
                    update_map |= 0b100
                if update_map == 0b111:
                    imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
                    self.publisher.publish(imu_msg)
                    update_map = 0b000


def main(args=None):
    rclpy.init(args=args)
    node = NaviGuider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
