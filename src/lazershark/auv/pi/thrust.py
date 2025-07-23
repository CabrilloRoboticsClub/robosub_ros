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

from std_msgs.msg import Int16MultiArray
from simple_pid import PID
from scipy.spatial.transform import Rotation
import numpy as np
from os import path
from scipy.optimize import curve_fit

PATH = path.dirname(__file__)

class Thrust(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__("thrust")

        self.error = [PID(0, 1, 0, setpoint=0) for _ in range(12)]

        # TODO: Make numbers good
        self.THRUST_MAX = 10

        com_pos = [0.20022, 0.06563, 0.0361]

        self.motor_positions = [ # [X, Y, Z] positions for each motors
            [ 0.19891, -0.30409, -0.06545], # Motor 0
            [ 0.19898,  0.17679, -0.06545], # Motor 1
            [-0.59640,  0.17686, -0.06545], # Motor 2
            [-0.59647, -0.30402, -0.06545], # Motor 3
            [ 0.08063, -0.28802, -0.00372], # Motor 4
            [ 0.08816,  0.15326, -0.00372], # Motor 5
            [-0.47812,  0.16079, -0.00372], # Motor 6
            [-0.48565, -0.28049, -0.00372]  # Motor 7
        ]

        for i in range(8):
            for j in range(3):
                self.motor_positions[i][j] += com_pos[j]

        # TODO: Check these numbers in pool (again)
        self.motor_thrusts = [ # [X, Y, Z] components of thrust for each motor
            [    0.0,     0.0,  1.0],   # Motor 0
            [    0.0,     0.0,  1.0],   # Motor 1
            [    0.0,     0.0,  1.0],   # Motor 2
            [    0.0,     0.0,  1.0],   # Motor 3
            [ 0.7071,  0.7071,  0.0],   # Motor 4
            [ 0.7071,  0.7071,  0.0],   # Motor 5
            [ 0.7071,  0.7071,  0.0],   # Motor 6
            [ 0.7071,  0.7071,  0.0],   # Motor 7
        ]

        # TODO: Generate feedback matrix in startup
        self.K = np.negative([
                    [ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.8037, 0.0,    0.0,    0.0,     0.0,     0.0],
                    [ 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,    5.8037, 0.0,    0.0,     0.0,     0.0],
                    [ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,    0.0,    5.8037, 0.0,     0.0,     0.0],
                    [ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,    0.0,    0.0,    2.2508,  0.1232,  0.0779],
                    [ 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,    0.0,    0.0,    0.1233,  2.0262, -0.2987],
                    [ 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,    0.0,    0.0,    0.0779, -0.2987,  2.4774],
        ])

        self.center_of_mass = [0.0] * 3

        self.motor_config = self.generate_motor_config(self.center_of_mass)
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)
        self.stab_map = np.matmul(self.inverse_config, self.K)

        self.pwm_fit_params = Thrust.generate_pwm_fit_params()

        self.pwm_pub = self.create_publisher(Int16MultiArray, "pwm_values", 10)
        # TODO: Make this take input from the EKF
        self.odom_sub = self.create_subscription(Odometry, "odometry/filtered", self.generate_motor_values, 10)

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
        rotation = Rotation.from_quat(quat).as_euler

        # Convert Twist to single vector for multiplication
        state = [
            0.0, # odometry.pose.pose.position.x,
            0.0, # odometry.pose.pose.position.y,
            0.0, # odometry.pose.pose.position.z,
            0.0, # rotation[0],                    # Roll
            0.0, # rotation[1],                    # Pitch
            0.0, # rotation[2],                    # Yaw
            odometry.twist.linear.x,
            odometry.twist.linear.y,
            0.0, # odometry.twist.linear.z,
            odometry.twist.angular.x,
            odometry.twist.angular.y,
            0.0, # odometry.twist.angular.z,
        ]
        for i in range(12):
            state[i] += self.error[i](state[i])

        # Multiply twist with inverse of motor config to get motor effort values
        motor_values = np.matmul(self.stab_map, state)

        scalar = self.THRUST_MAX / ((val:=max(motor_values)) + (val == 0))
        # Scalar will be the smaller of the two, largest value in twist array
        # will be percentage of that maximum
        thrust_values = [scalar * value for value in motor_values]

        pwm_values = Int16MultiArray()  
        pwm_values.data = [0] * 8
        motor_values = thrust_values
        for index, newton in enumerate(motor_values):
            pwm_values.data[index] = int(Thrust.newtons_to_pwm(
                newton,
                self.pwm_fit_params[0],
                self.pwm_fit_params[1],
                self.pwm_fit_params[2],
                self.pwm_fit_params[3],
                self.pwm_fit_params[4],
                self.pwm_fit_params[5]))
            pwm_values.data[index] = 1900 if pwm_values.data[index] > 1900 else 1100 if pwm_values.data[index] < 1100 else pwm_values.data[index]
            if newton == 0: pwm_values.data[index] = 1500
        self.pwm_pub.publish(pwm_values)
        

    @staticmethod
    def newtons_to_pwm(x: float, a: float, b: float, c: float, d: float, e: float, f: float) -> float:
        """
        Converts desired newtons into its corresponding PWM value

        Args:
            x: The force in newtons desired
            a-f: Arbitrary parameters to map newtons to pwm, see __generate_curve_fit_params()

        Returns:
            PWM value corresponding to the desired thrust
        """
        return (a * x**5) + (b * x**4) + (c * x**3) + (d * x**2) + (e * x) + f

    @staticmethod
    def generate_pwm_fit_params():
        x = []
        y = []

        with open(PATH + "/newtons_to_pwm.tsv", "r") as file:
            for data_point in file:
                data = data_point.split("\t")
                x.append(data[0])
                y.append(data[1])

        optimal_params, param_covariance = curve_fit(Thrust.newtons_to_pwm, x, y)
        return optimal_params
    
    def __del__(self):
        pwm_values = Int16MultiArray()
        pwm_values.data = [1500] * 8
        self.pwm_pub.publish(pwm_values)
    
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