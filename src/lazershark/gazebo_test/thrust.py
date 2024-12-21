import sys

import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from actuator_msgs.msg import Actuators

import numpy as np
from os import path

PATH = path.dirname(__file__)

class Thrust(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__("thrust")

        self.THRUST_MAX = 800

        # Positions for only the upward thrusters
        self.motor_positions = [ 
            [ 0.324,  0.276, -0.025], # Motor 0
            [ 0.324, -0.276, -0.025], # Motor 4
            [-0.324, -0.276, -0.025], # Motor 6
            [-0.324,  0.276, -0.025]  # Motor 7
        ]
        
        # Thrust directions for only the upward thrusters
        self.motor_thrusts = [ 
            [ 0.0,  0.0,  1.0],   # Motor 0
            [ 0.0,  0.0,  1.0],   # Motor 4
            [ 0.0,  0.0,  1.0],   # Motor 6
            [ 0.0,  0.0,  1.0]    # Motor 7
        ]

        self.center_of_mass = [0.0] * 3

        self.motor_config = self.generate_motor_config()
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)

        self.subscription = self.create_subscription(Twist, "desired_twist", self.thrust_callback, 10)
        self.motor_publishers= self.create_publisher(Actuators, "/thruster_values", 10)

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

    def generate_motor_values(self, twist_msg):
        """Called every time the twist publishes a message."""

        # Convert the X,Y,Z,R,P,Y to thrust settings for each motor. 
        motor_values = []

        # Convert Twist to single vector for multiplication
        twist_array = [
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.linear.z,
            twist_msg.angular.x,
            twist_msg.angular.y,
            twist_msg.angular.z
        ]

        if twist_array == [0, 0, 0, 0, 0, 0]:
            return [0.0 for motor in range(4)] # No thrust needed

        # Multiply twist with inverse of motor config to get motor effort values
        motor_values = np.matmul(self.inverse_config, twist_array).tolist()

        try:
            scalar = self.THRUST_MAX / max(abs(val) for val in motor_values) * max(abs(val) for val in twist_array)
        except ZeroDivisionError:
            scalar = 0.0

        # scale and return motor values
        return [thrust * scalar for thrust in motor_values]

    def thrust_callback(self, twist_msg):
        msg = Actuators()
        thrust_values = self.generate_motor_values(twist_msg)

        msg.velocity = thrust_values  # Example actuator values
        self.motor_publishers.publish(msg)

        
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
