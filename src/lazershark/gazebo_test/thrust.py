import sys 

import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import numpy as np
from scipy.spatial.transform import Rotation
from simple_pid import PID
from os import path

PATH = path.dirname(__file__)

class Thrust(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__("thrust")

        self.THRUST_MAX = 20

        self.motor_positions = [ # [X, Y, Z] positions for each motors
            [ 0.324,  0.276, -0.025], # Motor 0
            [ 0.229,  0.223, -0.005], # Motor 1
            [ 0.229, -0.223, -0.005], # Motor 2
            [-0.229, -0.223, -0.005], # Motor 3
            [ 0.324, -0.276, -0.025], # Motor 4
            [-0.229,  0.223, -0.005], # Motor 5
            [-0.324, -0.276, -0.025], # Motor 6
            [-0.324,  0.276, -0.025]  # Motor 7
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

        self.center_of_mass = [0.0] * 3

        self.motor_config = self.generate_motor_config()
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)

        self.pid = {
            "angular_x": PID(1, 0.1, 0.05, setpoint=3.14159),
            "angular_y": PID(1, 0.1, 0.05, setpoint=0),
            "angular_z": PID(1, 0.1, 0.05, setpoint=0)
        }

        self.twist_sub = self.create_subscription(Twist, "desired_twist", self.twist_callback, 10)
        self.twist_array = [0.0] * 6
        self.odom_sub = self.create_subscription(Odometry, "odometry/filtered", self.odom_callback, 10)
        self.motor_publishers = [0] * 8
        for i in range(8):
            self.motor_publishers[i] = self.create_publisher(Float64, f"/thruster_values/thruster_{i}", 10)

    def generate_motor_config(self):
        torques = np.cross(self.motor_positions, self.motor_thrusts)
        return [
            [thrust[0] for thrust in self.motor_thrusts], # Fx (N)
            [thrust[1] for thrust in self.motor_thrusts], # Fy (N)
            [thrust[2] for thrust in self.motor_thrusts], # Fz (N)
            [torque[0] for torque in torques],            # Rx (N*m)
            [torque[1] for torque in torques],            # Ry (N*m)
            [torque[2] for torque in torques]             # Rz (N*m)
        ]

    def generate_motor_values(self, val):
        """Called every time the twist publishes a message."""

        # Publish zero thrust unless there's input
        motor_values = [0] * 8
        angular_x = [0] * 8

        if self.twist_array != [0, 0, 0, 0, 0, 0]:
            # Multiply twist with inverse of motor config to get motor effort values
            motor_values = np.matmul(self.inverse_config, self.twist_array).tolist()
            # scale motor values
            scalar_1 = self.THRUST_MAX / max(abs(val) for val in motor_values) * max(abs(val) for val in self.twist_array)
            motor_values = [thrust * scalar_1 for thrust in motor_values]


        if val != 0:
            thing = [0, 0, 0, val, 0, 0]
            # Multiply twist with inverse of motor config to get motor effort values
            angular_x = np.matmul(self.inverse_config, thing).tolist()
            # scale motor values
            scalar_2 = self.THRUST_MAX / max(abs(val) for val in angular_x)
            if scalar_2 > 1:
                angular_x = [thrust * scalar_2 for thrust in angular_x]    

        # Multiply twist with inverse of motor config to get motor effort values

        # return motor values
        return np.add(motor_values, angular_x).tolist()    

    def odom_callback(self, odom):
        quat = [
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        ]
        rot = Rotation.from_quat(quat).as_euler("xyz")
        val = self.pid["angular_x"](rot[0])
        thrust_msg = Float64()
        thrust_values = self.generate_motor_values(-val)
        for i in range(8):
            thrust_msg.data = thrust_values[i]
            self.motor_publishers[i].publish(thrust_msg)


    def twist_callback(self, twist_msg):
        self.twist_array = [
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.linear.z,
            twist_msg.angular.x,
            twist_msg.angular.y,
            twist_msg.angular.z
        ]
        # thrust_msg = Float64()
        # thrust_values = self.generate_motor_values(twist_msg)
        # for i in range(8):
        #     thrust_msg.data = thrust_values[i]
        #     self.motor_publishers[i].publish(thrust_msg)
        
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