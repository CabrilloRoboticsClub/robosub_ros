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
import rclpy
import sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation


class Logic(Node):
    """
    Task Planning Node.
    """
    
    def __init__(self):
        """
        Initialize `logic` node.
        """
        super().__init__("logic")
        # Pose generated from fused sensor readings, in A50's coordinates
        self.measured_pose = Odometry()
        # Pose modified for immediatel input into LQR, transformed into fixed Robot Frame
        self.modified_pose = Odometry()
        # Desired pose in the A50's coordinates
        self.desired_pose = {"x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "heading": 0.0,
        }

        self.roll_index = 1
        self.roll_scale = -5

        self.pitch_index = 0
        self.pitch_scale = 5

        self.odom_sub = self.create_subscription(Odometry, "odometry/filtered", self.ekf_update, 10)
        self.dvl_sub = self.create_subscription(Odometry, "dvl/a50", self.dvl_update, 10)
        self.publisher = self.create_publisher(Odometry, "odometry/logic", 10)

    # Subscribers

    def ekf_update(self, odometry):
        # Convert Odometry Quaternion into euler angles
        quat = [odometry.pose.pose.orientation.x, 
                odometry.pose.pose.orientation.y, 
                odometry.pose.pose.orientation.z, 
                odometry.pose.pose.orientation.w,]
        rotation = Rotation.from_quat(quat).as_euler('xyz', degrees=True)

        # Scale angles for LQR
        roll  = rotation[self.roll_index] / self.roll_scale
        pitch = rotation[self.pitch_index] / self.pitch_scale

        # Match Roll/Pitch Angles
        self.measured_pose.pose.pose.orientation.x = roll
        self.measured_pose.pose.pose.orientation.y = pitch

        # Take altitude
        self.measured_pose.pose.pose.position.z = odometry.pose.pose.position.z

    def dvl_update(self, odometry):
        # Match Position from DVL
        self.measured_pose.pose.pose.position.x = odometry.pose.pose.position.x
        self.measured_pose.pose.pose.position.y = odometry.pose.pose.position.y

        # Match heading from DVL 
        self.measured_pose.pose.pose.orientation.z = odometry.pose.pose.orientation.x

        # Match linear velocities
        self.measured_pose.twist.twist.linear.x =  odometry.twist.twist.linear.x
        self.measured_pose.twist.twist.linear.y =  odometry.twist.twist.linear.y
        self.measured_pose.twist.twist.linear.z = -odometry.twist.twist.linear.z

    # Prepare Odometry for LQR

    def shifted_heading(self):
        shifted = self.measured_pose - self.desired_pose["heading"]
        if shifted < -180.0:
            shifted += 360.0
        elif shifted > 180.0:
            shifted -= 360.0
        self.modified_pose.pose.pose.orientation.z = shifted
    
    # Takes in x,y make sure they have desired offset subtracted
    def rotate_xy(self, x, y):
        # Convert heading to radians
        theta = self.measured_pose.pose.pose.orientation.heading * 2 * np.pi / 360
       
        xy_vec = np.array([x, y])

        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)],
        ])

        rotated_xy_vec = np.matmul(rotation_matrix, xy_vec)

        x_rot, y_rot = rotated_xy_vec[0], rotated_xy_vec[1]

        self.modified_pose.pose.pose.position.x, self.modified_pose.pose.pose.position.y = x_rot, y_rot

    # Helpers for task preperation
    
    def maintain_x(self):
        self.desired_pose["x"] = self.measured_pose.pose.pose.position.x
    
    def maintain_y(self):
        self.desired_pose["y"] = self.measured_pose.pose.pose.position.y

    def maintain_heading(self):
        self.desired_pose["heading"] = self.measured_pose.pose.pose.orientation.z

    def maintain_pose(self):
        self.maintain_x()
        self.maintain_y()
        self.maintain_heading()

    def clamp_xy(self):
        x, y = self.modified_pose.pose.pose.position.x, self.modified_pose.pose.pose.position.y
        x = max(min(x, 1.0), -1.0)
        y = max(min(y, 1.0), -1.0)
        self.modified_pose.pose.pose.position.x, self.modified_pose.pose.pose.position.y = x, y


    def update_msg(self):
        self.shifted_heading()

        x = self.measured_pose.pose.pose.position.x - self.desired_pose["x"]
        y = self.measured_pose.pose.pose.position.y - self.desired_pose["y"]

        self.rotate_xy(x, y)

        self.modified_pose. z = self.measured_pose.pose.pose.position.z - self.desired_pose["z"]


def main(args=None):
    rclpy.init(args=args)
    node = Logic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
