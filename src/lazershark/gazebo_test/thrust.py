"""
thrust.py

Calculate correct output of motors and output it on /drive/motors

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

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from simple_pid import PID
from scipy.spatial.transform import Rotation
import numpy as np
from os import path
from math import sin, cos

PATH = path.dirname(__file__)

class Thrust(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__("thrust")

        self.error = [
            PID(0, 0.00, 0, setpoint=0),
            PID(0, 0.00, 0, setpoint=0),
            PID(0, 0.05, 0, setpoint=0),
            PID(0, 0.00, 0, setpoint=0),
            PID(0, 0.00, 0, setpoint=0),
            PID(0, 0.00, 0, setpoint=0),
        ]

        # TODO: Make numbers good
        self.THRUST_MAX = 20

        self.motor_positions = [ # [X, Y, Z] positions for each motors
            [ 0.32378,  0.27625, -0.01738], # Motor 0
            [ 0.32379, -0.28832, -0.01738], # Motor 1
            [-0.32372, -0.28832, -0.01738], # Motor 2
            [-0.32372,  0.27625, -0.01738], # Motor 3
            [ 0.23416,  0.21738,  0.00437], # Motor 4
            [ 0.23416, -0.22946,  0.00437], # Motor 5
            [-0.23409, -0.22946,  0.00438], # Motor 6
            [-0.23409,  0.21739,  0.00438]  # Motor 7
        ]
        self.motor_thrusts = [ # [X, Y, Z] components of thrust for each motor
            [    0.0,     0.0,  1.0],   # Motor 0
            [-0.7071,  0.7071,  0.0],   # Motor 1
            [-0.7071, -0.7071,  0.0],   # Motor 2
            [ 0.7071, -0.7071,  0.0],   # Motor 3
            [    0.0,     0.0,  1.0],   # Motor 6
            [ 0.7071,  0.7071,  0.0],   # Motor 5
            [    0.0,     0.0,  1.0],   # Motor 6
            [    0.0,     0.0,  1.0]    # Motor 7
        ]

        # TODO: Generate feedback matrix in startup
        self.K = [
            [0.9999999999999966,0.0,0.0,0.0,0.0,0.0,1.8514345015162799,0.0,0.0,0.0,0.0,0.0,],
            [0.0,0.9999999999999966,0.0,0.0,0.0,0.0,0.0,1.8514345015162799,0.0,0.0,0.0,0.0,],
            [0.0,0.0,0.9999999999999966,0.0,0.0,0.0,0.0,0.0,1.8514345015162799,0.0,0.0,0.0,],
            [0.0,0.0,0.0,0.0952464259560727,0.8847384682005144,-0.45625208079393914,0.0,0.0,0.0,0.11078725281933952,0.9499899715194083,-0.5283747305579365,],
            [0.0,0.0,0.0,0.8847384682005135,0.13483386021331392,0.44615879799257824,0.0,0.0,0.0,0.9499899715194073,0.16845579675487224,0.54608616342995,],
            [0.0,0.0,0.0,-0.456252080793939,0.4461587979925783,0.7699197138306183,0.0,0.0,0.0,-0.5283747305579376,0.5460861634299506,1.0220759595190467,],
        ]

        self.K = [[-3 * val for val in row] for row in self.K]

        self.center_of_mass = [0.0] * 3

        self.motor_config = self.generate_motor_config(self.center_of_mass)
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)

        self.motor_publishers = [0] * 8
        for i in range(8):
            self.motor_publishers[i] = self.create_publisher(Float64, f"/thruster_values/thruster_{i}", 10)
        # TODO: Make this take input from the EKF
        self.odom_sub = self.create_subscription(Odometry, "odometry/sim", self.generate_motor_values, 10)

    # def closest_vector(self, state, )

    def generate_motor_config(self, center_of_mass_offset):
        """
        Generate the motor configuration matrix based on motor positions and thrust. Allows for
        a shifting center of mass, so the motor configuration can be regenerated dynamically to
        account for center of mass shifts when lifting objects.

        Returns:
            Motor configuration matrix based on motor orientation, position, and location of center of mass
        """
        shifted_positons = [(np.subtract(motor, center_of_mass_offset).tolist())
                            for motor in self.motor_positions]
        torques = np.cross(shifted_positons, self.motor_thrusts)

        return [
            [thrust[0] for thrust in self.motor_thrusts], # Fx (N)
            [thrust[1] for thrust in self.motor_thrusts], # Fy (N)
            [thrust[2] for thrust in self.motor_thrusts], # Fz (N)
            [torque[0] for torque in torques],            # Rx (N*m)
            [torque[1] for torque in torques],            # Ry (N*m)
            [torque[2] for torque in torques]             # Rz (N*m)
        ]

    def generate_motor_values(self, odometry):
        """Called every time the twist publishes a message."""

        # Convert the X,Y,Z,R,P,Y to thrust settings for each motor. 
        motor_values = []

        quat = [odometry.pose.pose.orientation.x, 
                odometry.pose.pose.orientation.y, 
                odometry.pose.pose.orientation.z, 
                odometry.pose.pose.orientation.w,]
        # TODO: Set desired heading to zero in quat
        rot = Rotation.from_quat(quat)
        rx, ry, rz = rot.as_euler("xyz")
        rotmatrix = np.eye(12)
        rotmatrix[0:3,0:3] = rot.as_matrix()
        corrected_map = np.matmul(self.inverse_config, other_map:=np.matmul(self.K, rotmatrix))

        # Convert Twist to single vector for multiplication
        state = [
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y,
            odometry.pose.pose.position.z,
            rx,                    # Roll
            ry,                    # Pitch
            rz,                    # Yaw
            odometry.twist.twist.linear.x,
            odometry.twist.twist.linear.y,
            odometry.twist.twist.linear.z,
            odometry.twist.twist.angular.x,
            odometry.twist.twist.angular.y,
            odometry.twist.twist.angular.z,
        ]

        for i in range(3):
           state[i] -= self.error[i](state[i])

        # Multiply twist with inverse of motor config to get motor effort values
        motor_values = np.matmul(corrected_map, state)
        self.get_logger().info(f"{np.matmul(other_map, state)}")

        scalar = 1 if ((val:=max(motor_values))<=self.THRUST_MAX) else self.THRUST_MAX / ((val == 0) + val)
        # Scalar will be the smaller of the two, largest value in twist array
        # will be percentage of that maximum
        thrust_values = [scalar * value for value in motor_values]

        # scale and return motor values
        thrust_msg = Float64()
        for i in range(8):
            thrust_msg.data = thrust_values[i]
            self.motor_publishers[i].publish(thrust_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Thrust()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()    


if __name__ == "__main__":
    main(sys.argv)