"""
piloted_thrust.py

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
from time import sleep
import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from std_msgs.msg import Int16MultiArray
from simple_pid import PID
from scipy.spatial.transform import Rotation
import numpy as np
from os import path
from scipy.optimize import curve_fit
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

PATH = path.dirname(__file__)

class PilotedThrust(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__("piloted_thrust")

        # TODO: Make numbers good
        self.THRUST_MAX = 20

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
            [ 0.7071, -0.7071,  0.0],   # Motor 5
            [ 0.7071,  0.7071,  0.0],   # Motor 6
            [ 0.7071, -0.7071,  0.0],   # Motor 7
        ]

        self.center_of_mass = [0.0] * 3
        self.TOTAL_CURRENT_LIMIT = 12
        self.thrust_fit_params = Thrust.generate_thrust_fit_param()

        self.motor_config = self.generate_motor_config(self.center_of_mass)
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)

        self.pwm_fit_params = Thrust.generate_pwm_fit_params()

        self.pwm_pub = self.create_publisher(Int16MultiArray, "pwm_values", 10)
        self.twist_sub = self.create_subscription(Twist, "desired_twist", self.generate_motor_values, 10)

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

    def generate_motor_values(self, twist):
        """Called every time the twist publishes a message."""

        twist = [
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        ]

        twist = [self.THRUST_MAX * val for val in twist]

        motor_values = np.matmul(self.inverse_config, twist)

        if all(val == 0.0 for val in motor_values):
            pwm_values = Int16MultiArray()  
            pwm_values.data = [1500] * 8
            self.pwm_pub.publish(pwm_values)
            return

        scalar = self.get_current_scalar_value(motor_values, self.TOTAL_CURRENT_LIMIT)
        if scalar < 1.0:
            motor_values = [scalar * value for value in motor_values]

        pwm_values = Int16MultiArray()  
        pwm_values.data = [0] * 8
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
    
    def get_polynomial_coef(self, mv: list, limit: float) -> list:
        """
        Generates a list of the coefficients for a polynomial, the input of which is the
        motor scaling factor and the roots of the function are the maximum scaling factor.

        Args:
            mv: The motor values in newtons that when produced will result in our desired twist
            limit: The current limit we would like to stay under in amperes (TOTAL_CURRENT_LIMIT or ESC_CURRENT_LIMIT)

        Returns:        self.control_axis = self.get_parameter("control_axis").get_parameter_value().bool_array_value

            A list of the coefficients of a 5th degree polynomial function, where the input of said
            function is the scaling factor and the output is the current (A) draw
        """
        return [self.thrust_fit_params[0] * sum([thrust**6 for thrust in mv]),
                self.thrust_fit_params[1] * sum([thrust**5 for thrust in mv]),
                self.thrust_fit_params[2] * sum([thrust**4 for thrust in mv]),
                self.thrust_fit_params[3] * sum([thrust**3 for thrust in mv]),
                self.thrust_fit_params[4] * sum([thrust**2 for thrust in mv]),
                self.thrust_fit_params[5] * sum(mv),
                self.thrust_fit_params[6] * len(mv) - limit]


    def get_current_scalar_value(self, mv: list, limit: float) -> float:
        """
        Given a motor value list and a current limit, return the best scaling factor

        Args:
            mv: The motor values in newtons that when produced will result in our desired twist
            limit: The current limit we would like to stay under in amperes (TOTAL_CURRENT_LIMIT or ESC_CURRENT_LIMIT)

        Returns:
            A valid scaling factor
        """
        # Get coefficients for function given the motor values given and the current (Amp) limits
        coef_list = self.get_polynomial_coef(mv, limit)
        # Find roots
        potential_scaling_factors = np.roots(coef_list).tolist()
        # Ignore nonreal and negative scaling factors
        real_positive = [scalar.real for scalar in potential_scaling_factors if scalar.imag == 0 and scalar.real >= 0]
        # Return valid roots
        return max(real_positive)


    @staticmethod
    def __thrust_to_current(x: float, a: float, b: float, c: float, d: float, e: float, f: float, g: float) -> float:
        """
        Estimates current draw based on given thrust

        Args:
            x: Thrust being produced in newtons.
            a-f: Arbitrary parameters to map thrust to current, see generate_thrust_fit_params()

        Returns:
            Current (estimated) to be drawn in amps.
        """
        return (a * x**6) + (b * x**5) + (c * x**4) + (d * x**3) + (e * x**2) + (f * x) + (g)

    
    @staticmethod
    def generate_thrust_fit_param() -> list:
        """
        Generates Optimal Parameters for __thrust_to_current() to have a best fit

        Returns:
            List of optimal parameters
        """
        x = list()
        y = list()

        with open(PATH + "/thrust_to_current.tsv", "r") as file:
            for data_point in file:
                data = data_point.split("\t")
                x.append(data[0])
                y.append(data[1])

        optimal_params, param_covariance = curve_fit(Thrust.__thrust_to_current, x, y)
        return optimal_params


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
    node = PilotedThrust()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)