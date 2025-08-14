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
                    [ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.8037, 0.0,    0.0,    0.0,     0.0,     0.0],
                    [ 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,    5.8037, 0.0,    0.0,     0.0,     0.0],
                    [ 0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0.0,    0.0,    5.8037, 0.0,     0.0,     0.0],
                    [ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,    0.0,    0.0,    2.2508,  0.1232,  0.0779],
                    [ 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,    0.0,    0.0,    0.1233,  2.0262, -0.2987],
                    [ 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,    0.0,    0.0,    0.0779, -0.2987,  2.4774],
        ])

        # print(self.K)

        self.center_of_mass = [0.0] * 3
        self.TOTAL_CURRENT_LIMIT = 12
        self.thrust_fit_params = Thrust.generate_thrust_fit_param()

        self.declare_parameter("stabilization_position", [0.0, 0.0, -0.3])
        self.declare_parameter("control_axis", [False, False, True])

        self.stab_pos = self.get_parameter("stabilization_position").get_parameter_value().double_array_value.tolist()
        self.control_axis = self.get_parameter("control_axis").get_parameter_value().bool_array_value

        print(self.stab_pos)
        print(self.control_axis)
        # self.add_on_set_parameters_callback(self.update_control_params)

        self.motor_config = self.generate_motor_config(self.center_of_mass)
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)
        self.stab_map = np.matmul(self.inverse_config, self.K)

        self.pwm_fit_params = Thrust.generate_pwm_fit_params()

        self.pwm_pub = self.create_publisher(Int16MultiArray, "pwm_values", 10)
        # TODO: Make this take input from the EKF
        self.odom_sub = self.create_subscription(Odometry, "odometry/filtered", self.generate_motor_values, 10)
        self.dvl_sub = self.create_subscription(Odometry, "dvl/a50", self.update_heading, 10)
        self.heading = 0.0
        self.xy = [0.0, 0.0]
        self.stage = 0

        print("Working")
        for i in range(10):
            self.get_logger().info(f"time: {10 - i}")
            sleep(1)
        self.initial_rejection = True



    def update_heading(self, odometry):
        self.xy[0] = odometry.pose.pose.position.x
        self.xy[1] = odometry.pose.pose.position.y
        self.heading = odometry.pose.pose.orientation.x

    # def update_control_params(self, params: list[Parameter]) -> SetParametersResult:
    #     """
    #     Callback for parameter update.

    #     Args:
    #         params: List of updated parameters (handles by ROS2)

    #     Returns:
    #         SetParametersResult() which lets ROS2 know if the parameters were set correctly or not
    #     """


    #     # Where `center_of_mass_increment` is a param set by either `pilot_input` or `dash` 
    #     for param in params:
    #         if param.name == "stabilization_position":
    #             if len(value:=param.value.tolist()) == 3:
    #                 if (value == [0.0] * 3):
    #                     self.center_of_mass = value
    #                 else:
    #                     for i, inc in enumerate(value):
    #                         self.center_of_mass[i] += inc
    #                 self.motor_config = self.generate_motor_config(self.center_of_mass)
    #                 self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)
    #                 return SetParametersResult(successful=True)
    #     return SetParametersResult(successful=False)

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

        self.stab_pos = self.get_parameter("stabilization_position").get_parameter_value().double_array_value.tolist()
        self.control_axis = self.get_parameter("control_axis").get_parameter_value().bool_array_value

        motor_values = []

        quat = [odometry.pose.pose.orientation.x, 
                odometry.pose.pose.orientation.y, 
                odometry.pose.pose.orientation.z, 
                odometry.pose.pose.orientation.w,]
        # TODO: Set desired heading to zero in quat
        rotation = Rotation.from_quat(quat).as_euler('xyz', degrees=True)

        if (rotation[0] > 0):
            rotation[0] -= 180
        else:
            rotation[0] += 180

        print(f"Position X: {self.xy[0]}")
        print(f"Position Y: {self.xy[1]}")
        print(f"Position Z: {odometry.pose.pose.position.z}")
        print(f"Roll: {-rotation[1]}")
        print(f"Pitch: {rotation[0]}")
        print(f"Yaw: {self.heading}")
        print(f"Velocity X: {odometry.twist.twist.linear.x}")
        print(f"Velocity Y: {odometry.twist.twist.linear.y}")
        print(f"Velocity Z: {odometry.twist.twist.linear.z}")
        print(f"Roll speed: {odometry.twist.twist.angular.y}")
        print(f"Pitch speed: {odometry.twist.twist.angular.x}")
        print(f"Yaw speed: {-odometry.twist.twist.angular.z}")
        print("---------------------")

        # rotation[1] += 30
        rotation[2] += 100

        roll  = rotation[1] / -5
        pitch = rotation[0] / 5
        yaw   = self.heading / 5

        if abs(pitch) > 30.0 and self.initial_rejection:
            return
        self.initial_rejection = False

        # 1: 7.5 forward

        # 2: 2 left

        # 3: 1.5 back

        # 4: 2 right

        # 5: 6 back

        # Convert Twist to single vector for multiplication
        z = odometry.pose.pose.position.z
        # print(self.stage)
        # if self.stage == 0:
        #     x = 0.0
        #     y = 0.0
        #     if z < 0.6:
        #         self.stage = 1
        # elif self.stage == 1:
        #     x = -10.0 if self.xy[0] < 8.0 else 0.0
        #     y = self.xy[1]
        #     if x == 0.0:
        #         self.stage = 2
        # elif self.stage == 2:
        #     x = self.xy[0] - 8.0
        #     y = -10.0 if self.xy[1] < 2.0 else 0.0
        #     if y == 0.0:
        #         self.stage = 3
        # elif self.stage == 3:
        #     x = 10.0 if self.xy[0] > 6.0 else 0.0
        #     y = self.xy[1] - 2.0
        #     if x == 0.0:
        #         self.stage = 4
        # elif self.stage == 4:
        #     x = self.xy[0] - 6.0
        #     y = 10.0 if self.xy[1] > 0.0 else 0.0
        #     if y == 0.0:
        #         self.stage = 5
        # else:
        #     x = 10.0 if self.xy[0] > 0.2 else 0.0
        #     y = self.xy[1]
        #     if x == 0.0:
        #         pwm_values = Int16MultiArray()  
        #         pwm_values.data = [1500] * 8
        #         self.pwm_pub.publish(pwm_values)
        #         sys.exit(0)



        state = [
            0.0,#x, #-10.0 if self.xy[0] < 7.5 else 0.0,#odometry.pose.pose.position.x - 13.0, # odometry.pose.pose.position.x - self.stab_pos[0] if self.control_axis[0] else 0.0,
            0.0,#y, #self.xy[1],#-(odometry.pose.pose.position.y - 1.0), #self.stab_pos[1] if self.control_axis[1] else 0.0,
            z - 0.5, #odometry.pose.pose.position.z - 0.5,
            roll,                    # Roll
            pitch,                    # Pitch
            yaw,                    # Yaw
            0.0, #odometry.twist.twist.linear.x,
            0.0, #odometry.twist.twist.linear.y,
            0.0, #odometry.twist.twist.linear.z,
            odometry.twist.twist.angular.y,
            odometry.twist.twist.angular.x,
            -odometry.twist.twist.angular.z,
        ]

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
