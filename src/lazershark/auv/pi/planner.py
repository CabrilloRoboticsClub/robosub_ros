# """
# planner.py

# Handle all pilot input

# Copyright (C) 2022-2023 Cabrillo Robotics Club

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.

# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# Cabrillo Robotics Club
# 6500 Soquel Drive Aptos, CA 95003
# cabrillorobotics@gmail.com
# """
# # For reading argv
import sys 
import os
import yaml
from enum import Enum

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry 

from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import numpy as np


class State():
    def __init__(self, state=0, pose=0):
        self._state = state
        self._pose = pose
        self._tasks = [
            "start",
            # "spin",
            "gate",
            # "channel",
            # "dropper",
            # "torpedo",
            # "octagon",
            # "home",
        ]
    
    def next_state(self):
        self._state += 1
        self._pose = 0

    def next_pose(self):
        self._pose += 1
    
    def set_state(self, state=0, pose=0):
        self._state = state
        self._pose = pose

    def s_str(self):
        return self._tasks[self._state]
    
    def s_int(self):
        return self._state
    
    def p_int(self):
        return self._pose
    


class Planner(Node):
    def __init__(self):
        super().__init__("planner")
        self.declare_parameter("pathA", "Not set.")
        self.declare_parameter("default_z", 0.8)

        self.odom_sub =  self.create_subscription(Odometry, "odometry/filtered", self.ekf_callback, 10)
        self.dvl_sub =   self.create_subscription(Odometry, "dvl/a50", self.dvl_callback, 10)
        self.planner_pub = self.create_publisher(Odometry,    "odometry/planner", 10)
    
        pkg_lazershark = get_package_share_directory("lazershark")
        yaml_path = os.path.join(
            pkg_lazershark, "params", self.get_parameter("path").value
        )

        self.default_z = self.get_parameter("default_z").value

        with open(yaml_path, "r") as file:
            self.path = yaml.load(file, Loader=yaml.SafeLoader)

        self.state = State(state=0)
        self.desired_pose:dict[float, float] = self.path[self.state.s_str()][self.state.p_int()]

        # Pose generated from fused sensor readings, in A50's coordinates
        self.measured_pose = Odometry()
        # Pose modified for immediatel input into LQR, transformed into fixed Robot Frame
        self.modified_pose = Odometry()

    def update_pose(self):
        self.state.next_pose()
        if len(self.path[self.state.s_str()]) == self.state.p_int():
            self.state.next_state()
    
    def pose_achieved(self) -> bool:
        return (abs(self.measured_pose.pose.pose.position.x - self.desired_pose["x"]) < 0.1
                and abs(self.measured_pose.pose.pose.position.y - self.desired_pose["y"]) < 0.1)
    
    def publish_plan(self):
        self.modified_pose = self.measured_pose

        self.modified_pose.pose.pose.position.x -= self.desired_pose["x"]
        self.modified_pose.pose.pose.position.y -= self.desired_pose["y"]
        if "z" in self.desired_pose:
            self.modified_pose.pose.pose.position.z -= self.desired_pose["z"]
        else:
            self.modified_pose.pose.pose.position.z -= self.default_z
        
        self.clamp_xy()
    
        self.publish_plan.publish(self.modified_pose)

    def ekf_callback(self, odometry):
        # Convert Odometry Quaternion into euler angles
        quat = [odometry.pose.pose.orientation.x, 
                odometry.pose.pose.orientation.y, 
                odometry.pose.pose.orientation.z, 
                odometry.pose.pose.orientation.w,]
        rotation = Rotation.from_quat(quat).as_euler('xyz', degrees=True)

        # Scale angles for LQR
        roll  = rotation[self.roll_index] / self.roll_scale
        pitch = rotation[self.pitch_index] / self.pitch_scale

        if (pitch > 0):
            pitch -= 180
        else:
            pitch += 180


        # Match Roll/Pitch Angles
        self.measured_pose.pose.pose.orientation.x = roll
        self.measured_pose.pose.pose.orientation.y = pitch

        # Take altitude
        self.measured_pose.pose.pose.position.z = odometry.pose.pose.position.z

        self.measured_pose.twist.twist.angular.x =  odometry.twist.twist.angular.x
        self.measured_pose.twist.twist.angular.y =  odometry.twist.twist.angular.y  
        self.measured_pose.twist.twist.angular.z = -odometry.twist.twist.angular.z

        if self.pose_achieved():
            self.update_pose()
        self.publish_plan()

    def dvl_callback(self, odometry):
        # Match Position from DVL
        self.measured_pose.pose.pose.position.x = odometry.pose.pose.position.x
        self.measured_pose.pose.pose.position.y = odometry.pose.pose.position.y

        # Match heading from DVL 
        self.measured_pose.pose.pose.orientation.z = odometry.pose.pose.orientation.x

        # Match linear velocities
        self.measured_pose.twist.twist.linear.x =  odometry.twist.twist.linear.x
        self.measured_pose.twist.twist.linear.y =  odometry.twist.twist.linear.y
        self.measured_pose.twist.twist.linear.z = -odometry.twist.twist.linear.z

    # TODO: Fix the desired pose
    def shifted_heading(self):
        """
        Sets heading such that the zero point is at the desired heading
        """
        shifted = self.measured_pose - self.desired_pose["heading"]
        if shifted < -180.0:
            shifted += 360.0
        elif shifted > 180.0:
            shifted -= 360.0
        self.modified_pose.pose.pose.orientation.z = shifted
    
    def rotate_xy(self, x, y):
        """
        Rotates the pose in the dvl's coordinates into the robot's coordinates.
        """
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

    # def maintain_x(self):
    #     self.desired_pose["x"] = self.measured_pose.pose.pose.position.x
    
    # def maintain_y(self):
    #     self.desired_pose["y"] = self.measured_pose.pose.pose.position.y

    # def maintain_heading(self):
    #     self.desired_pose["heading"] = self.measured_pose.pose.pose.orientation.z

    # TODO: Make this better later, for now this keeps x and y values between -0.1 and 0.1
    def clamp_xy(self):
        x, y = self.modified_pose.pose.pose.position.x, self.modified_pose.pose.pose.position.y
        # Clamp value [-0.1, 0.1]
        x = max(min(x, 0.1), -0.1)
        y = max(min(y, 0.1), -0.1)
        self.modified_pose.pose.pose.position.x, self.modified_pose.pose.pose.position.y = x, y

    
def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)