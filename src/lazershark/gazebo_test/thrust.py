import sys 

import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

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

        self.THRUST_MAX = 200

        self.motor_positions = [ # [X, Y, Z] positions for each motors
            [ 0.200,  0.130,  0.004], # Motor 0
            [ 0.200, -0.130,  0.047], # Motor 1
            [-0.200,  0.130,  0.047], # Motor 2
            [-0.200, -0.130,  0.047], # Motor 3
            [ 0.198,  0.156, -0.038], # Motor 4
            [ 0.198, -0.156, -0.038], # Motor 5
            [-0.198,  0.156, -0.038], # Motor 6
            [-0.198, -0.156, -0.038]  # Motor 7
        ]
        self.motor_thrusts = [ # [X, Y, Z] components of thrust for each motor
            [    0.0,     0.0,  1.0],   # Motor 0
            [    0.0,     0.0,  1.0],   # Motor 1 
            [    0.0,     0.0,  1.0],   # Motor 2
            [    0.0,     0.0,  1.0],   # Motor 3
            [-0.7071,  0.7071,  0.0],   # Motor 4
            [-0.7071, -0.7071,  0.0],   # Motor 5
            [ 0.7071,  0.7071,  0.0],   # Motor 6
            [ 0.7071, -0.7071,  0.0]    # Motor 7
        ]

        self.center_of_mass = [0.0] * 3

        self.motor_config = self.generate_motor_config()
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)

        self.subscription = self.create_subscription(Twist, "desired_twist", self.thrust_callback, 10)
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
            return [0.0 for motor in range(8)] # No thrust needed

        # Multiply twist with inverse of motor config to get motor effort values
        motor_values = np.matmul(self.inverse_config, twist_array).tolist()

        scalar = self.THRUST_MAX / min(motor_values)

        # scale and return motor values
        return [thrust * scalar for thrust in motor_values]

    def thrust_callback(self, twist_msg):
        thrust_msg = Float64()
        thrust_values = self.generate_motor_values(twist_msg)
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