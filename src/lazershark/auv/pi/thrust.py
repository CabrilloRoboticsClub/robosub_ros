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
from time import sleep
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
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

PATH = path.dirname(__file__)

class Thrust(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__("thrust")

        self.error = [
            # Position / Orientation
                PID(0.0,   0.0, 0.0, setpoint=0),   # X
                PID(0.0,   0.0, 0.0, setpoint=0),   # Y
                PID(0.0, -0.025, 0.0, setpoint=0),   # Z
                PID(0.0, -0.05, 0.0, setpoint=0),   # Roll
                PID(0.0, -0.05, 0.0, setpoint=0),   # Pitch
                PID(0.0, -0.02, 0.0, setpoint=0),   # Yaw
            # Velocities
                PID(0.0,   0.0, 0.0, setpoint=0),   # X
                PID(0.0,   0.0, 0.0, setpoint=0),   # Y
                PID(0.0,   0.0, 0.0, setpoint=0),   # Z
                PID(0.0,   0.0, 0.0, setpoint=0),   # Roll
                PID(0.0,   0.0, 0.0, setpoint=0),   # Pitch
                PID(0.0,   0.0, 0.0, setpoint=0),   # Yaw
        ]

        # TODO: Make numbers good
        self.THRUST_MAX = 15

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
            [    0.0,     0.0,  1.0],   # Motor 2 # Stationary bottom of pool .1 deg / 2 min  36.95 deg -> 37.05 deg
            [    0.0,     0.0,  1.0],   # Motor 3 # Stationary bottom of pool james stirring water .56deg / 2 min 37.06 deg -> 36.5 deg, james shifted slightly
            [ 0.7071,  0.7071,  0.0],   # Motor 4 # Stationary bottom of pool z motors down .45 deg / 2 min, 36.36 deg -> 36.81 deg
            [ 0.7071, -0.7071,  0.0],   # Motor 5 # Stationary bottom of pool xy motors forward 1.6 deg / 2min, 36.25 deg -> 37.8 deg, james shifted slightly 
            [ 0.7071,  0.7071,  0.0],   # Motor 6 # Stationary bottom of pool motor 7 max forward
            [ 0.7071, -0.7071,  0.0],   # Motor 7 # 38.06
        ]

        # TODO: Generate feedback matrix in startup
        self.K = np.negative([
                    [ 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.8037, 0.0,    0.0,    0.0,     0.0,     0.0],
                    [ 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0,    5.8037, 0.0,    0.0,     0.0,     0.0],
                    [ 0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0.0,    0.0,    5.8037, 0.0,     0.0,     0.0],
                    [ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,    0.0,    0.0,    2.2508,  0.1232,  0.0779],
                    [ 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,    0.0,    0.0,    0.1233,  2.0262, -0.2987],
                    [ 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,    0.0,    0.0,    0.0779, -0.2987,  2.4774],
        ])

        # print(self.K)

        self.center_of_mass = [0.0] * 3
        self.TOTAL_CURRENT_LIMIT = 12
        self.thrust_fit_params = Thrust.generate_thrust_fit_param()

        self.motor_config = self.generate_motor_config(self.center_of_mass)
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)
        self.stab_map = np.matmul(self.inverse_config, self.K)

        self.pwm_fit_params = Thrust.generate_pwm_fit_params()

        self.pwm_pub = self.create_publisher(Int16MultiArray, "pwm_values", 10)
        # TODO: Make this take input from the EKF
        self.odom_sub = self.create_subscription(Odometry, "odometry/planner", self.generate_motor_values, 10)

        print("Working")
        for i in range(10):
            self.get_logger().info(f"time: {10 - i}")
            sleep(1)
        self.initial_rejection = True

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

        state = [
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y,
            odometry.pose.pose.position.z,
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.twist.twist.linear.x,
            odometry.twist.twist.linear.y,
            odometry.twist.twist.linear.z,
            odometry.twist.twist.angular.x,
            odometry.twist.twist.angular.y,
            odometry.twist.twist.angular.z,
        ]

        print(f"Position X: {state[0]}")
        print(f"Position Y: {state[1]}")
        print(f"Position Z: {state[2]}")
        print(f"Roll: {state[3]}")
        print(f"Pitch: {state[4]}")
        print(f"Yaw: {state[5]}")
        print(f"Velocity X: {state[6]}")
        print(f"Velocity Y: {state[7]}")
        print(f"Velocity Z: {state[8]}")
        print(f"Roll speed: {state[9]}")
        print(f"Pitch speed: {state[10]}")
        print(f"Yaw speed: {state[11]}")
        print("---------------------")


        if abs(state[4]) > 30.0 and self.initial_rejection:
            return
        self.initial_rejection = False


        for i in range(12):
            if i != 5 and i != 0:
                state[i] += self.error[i](state[i])

        # print(state)

        print(f"{np.matmul(self.K, state)}")
        # Multiply twist with inverse of motor config to get motor effort values
        motor_values = np.matmul(self.stab_map, state)

        scalar = self.get_current_scalar_value(motor_values, self.TOTAL_CURRENT_LIMIT)
        if scalar < 1.0:
            print(f"Estimated Current: {sum(self.get_polynomial_coef(motor_values, 0.0))}")
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
        # pwm_values.data = [1500] * 8
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
    node = Thrust()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()    


if __name__ == "__main__":
    main(sys.argv)
