# For reading argv
import sys 

# ROS client library imports
import rclpy
from rclpy.node import Node 

# ROS messages imports
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class PilotInput(Node):
    """
    Class that implements the joystick input
    """

    def __init__(self):
        """
        Initialize 'pilot_input' node
        """
        super().__init__("pilot_input")

        # Create publishers and subscriptions
        self.subscription = self.create_subscription(Joy, "joy", self.callback, 10)
        self.twist_pub = self.create_publisher(Twist, "desired_twist", 10)

    def callback(self, joy_msg: Joy):
        """
        Takes in input from the joy message from the x box and republishes it as a twist specifying 
        the direction (linear and angular x, y, z) and percent of max speed the pilot wants the robot to move

        Args:
            joy_msg: Message of type 'Joy' from the joy topic
        """

        # Debug output of joy topic
        self.get_logger().debug(f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}")

        # Map the values sent from the joy message to useful names
        controller = {
            # Left stick
            "linear_x":        -joy_msg.axes[0],                # left_stick_x
            "linear_y":        -joy_msg.axes[1],                # left_stick_y
            # "":               joy_msg.buttons[9],             # left_stick_press
            # Right stick
            "angular_z":        joy_msg.axes[3],                # right_stick_x
            "angular_x":       -joy_msg.axes[4],                # right_stick_y
            # "":               joy_msg.buttons[10],            # right_stick_press
            # Triggers
            "neg_linear_z":     joy_msg.axes[2],                # left_trigger
            "pos_linear_z":     joy_msg.axes[5],                # right_trigger
            # Dpad
            # "":               joy_msg.axes[7],                # dpad up/dowm (used for tilty thing)
            # "":               joy_msg.axes[6]                 # dpad_left/right  (used for spinny thing)
            # Buttons
            # "back_claw":        joy_msg.buttons[0], # a
            # "bambi_mode":       joy_msg.buttons[1], # b
            # "toggle_claw":      joy_msg.buttons[2], # x
            # "articulate_claw":  joy_msg.buttons[3], # y
            "pos_angular_y":    joy_msg.buttons[4], # left_bumper
            "neg_angular_y":    joy_msg.buttons[5], # right_bumper
            # "kill":             joy_msg.buttons[6], # window
            # "reverse":          joy_msg.buttons[7], # menu
            # "reset":            joy_msg.buttons[8], # xbox
        }

        # Create twist message
        twist_msg = Twist()

        twist_msg.linear.x  = controller["linear_x"]     # forwards
        twist_msg.linear.y  = -controller["linear_y"]   # sideways
        twist_msg.linear.z  = ((controller["neg_linear_z"] - controller["pos_linear_z"]) / 2)    # depth
        twist_msg.angular.x = controller["angular_x"]    # pitch
        twist_msg.angular.y = -(controller["pos_angular_y"] - controller["neg_angular_y"]) * 0.5 # roll (const +/- 0.5 thrust)
        twist_msg.angular.z = controller["angular_z"]    # yaw

        # Publish twist message
        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PilotInput())
    rclpy.shutdown()


if __name__ == "main":
    main(sys.argv)