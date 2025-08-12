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
                PID(0.0, -0.05, 0.0, setpoint=0),   # Z
                PID(0.0, -0.05, 0.0, setpoint=0),   # Roll
                PID(0.0, -0.05, 0.0, setpoint=0),   # Pitch
                PID(0.0, -0.05, 0.0, setpoint=0),   # Yaw
            # Velocities
                PID(0.0,   0.0, 0.0, setpoint=0),   # X
                PID(0.0,   0.0, 0.0, setpoint=0),   # Y
                PID(0.0,   0.0, 0.0, setpoint=0),   # Z
                PID(0.0,   0.0, 0.0, setpoint=0),   # Roll
                PID(0.0,   0.0, 0.0, setpoint=0),   # Pitch
                PID(0.0,   0.0, 0.0, setpoint=0),   # Yaw
        ]

        # TODO: Make numbers good
        self.THRUST_MAX = 5

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

        # TODO: Generate feedback matrix in startup
        self.K = np.negative([
                    [ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.8037, 0.0,    0.0,    0.0,     0.0,     0.0],
                    [ 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,    5.8037, 0.0,    0.0,     0.0,     0.0],
                    [ 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0,    0.0,    5.8037, 0.0,     0.0,     0.0],
                    [ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,    0.0,    0.0,    2.2508,  0.1232,  0.0779],
                    [ 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,    0.0,    0.0,    0.1233,  2.0262, -0.2987],
                    [ 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,    0.0,    0.0,    0.0779, -0.2987,  2.4774],
        ])

        # print(self.K)

        self.center_of_mass = [0.0] * 3

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
        self.odom_sub = self.create_subscription(Odometry, "dvl/a50", self.generate_motor_values, 10)

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

        print(f"Position X: {odometry.pose.pose.position.x}")
        print(f"Position Y: {odometry.pose.pose.position.y}")
        print(f"Position Z: {odometry.pose.pose.position.z}")
        print(f"Roll: {-odometry.pose.pose.orientation.x}")
        print(f"Pitch: {odometry.pose.pose.orientation.y}")
        print(f"Yaw: {odometry.pose.pose.orientation.z}")
        print(f"Velocity X: {odometry.twist.twist.linear.x}")
        print(f"Velocity Y: {odometry.twist.twist.linear.y}")
        print(f"Velocity Z: {odometry.twist.twist.linear.z}")
        print(f"Roll speed: {odometry.twist.twist.angular.x}")
        print(f"Pitch speed: {odometry.twist.twist.angular.y}")
        print(f"Yaw speed: {odometry.twist.twist.angular.z}")
        print("---------------------")

        # rotation[1] += 30

        odometry.pose.pose.orientation.x /= 5
        odometry.pose.pose.orientation.y /= 5
        odometry.pose.pose.orientation.z /= 5



        # Convert Twist to single vector for multiplication
        state = [
            0.0, # odometry.pose.pose.position.x - self.stab_pos[0] if self.control_axis[0] else 0.0,
            0.0, # odometry.pose.pose.position.y - self.stab_pos[1] if self.control_axis[1] else 0.0,
            odometry.pose.pose.position.z - 0.5, #
            0.0,#    -odometry.pose.pose.orientation.x,#rotation[1],                    # Roll
            0.0, #     odometry.pose.pose.orientation.y,                    # Pitch
            0.0,#rotation[2],                    # Yaw
            odometry.twist.twist.linear.x,
            odometry.twist.twist.linear.y,
            odometry.twist.twist.linear.z,
            0.0, # odometry.twist.twist.angular.y,
            0.0, # odometry.twist.twist.angular.x,
            0.0, # -odometry.twist.twist.angular.z,
        ]

        for i in range(12):
            state[i] += self.error[i](state[i])

        # print(state)

        print(f"{np.matmul(self.K, state)}")
        # Multiply twist with inverse of motor config to get motor effort values
        motor_values = np.matmul(self.stab_map, state)

        scalar = 1
        if max(abs(motor_values)) > self.THRUST_MAX:
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
        # pwm_values.data = [1500] * 8
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
