"""
pilot_input.py

Handle all pilot input

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
# For reading argv
import sys 

# ROS client library imports
import rclpy
from rclpy.node import Node 

# ROS messages imports
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class StickyButton():
    """
    Class that implements sticky buttons, meaning a button is pressed to turn it on, 
    and pressed again to turn it off
    """

    def __init__(self):
        """
        Initialize 'StickyButton' object
        """
        self.__feature_state = False    # Is the feature this button controls on (True) or off (False)
        self.__track_state = 0b0000     # Tracks the last four states of the button using bits
    
    def check_state(self, cur_button_state: bool) -> bool:
        """
        Checks if a button is toggled on or off and accounts for debouncing

        Args:
            cur_button_state: Current state of the button provided by the controller

        Returns:
            True if the button is toggled on, False if off
        """
        # Append the current button state to the tracker then removes all but rightmost byte
        # such that self.__track_state records the most recent four states of the button
        self.__track_state = (self.__track_state << 1 | cur_button_state) & 0b1111

        # Account for bounce by making sure the last four recorded states
        # appear to represent a button press. A debounced button press is defined as two recorded
        # consecutive off states followed by two on states. If the button is pressed, update the feature state
        if self.__track_state == 0b0011:
            self.__feature_state = not self.__feature_state
        return self.__feature_state

    def reset(self):
        """
        Resets button state to original configuration
        """
        self.__feature_state = False
        self.__track_state = 0b0000


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

        # Button mapping
        self.buttons = {
            # "" :              StickyButton(),     # left_stick_press
            # "" :              StickyButton(),     # right_stick_press
            "back_claw" :       StickyButton(),     # a
            "bambi_mode":       StickyButton(),     # b
            "toggle_claw":      StickyButton(),     # x
            "articulate_claw":  StickyButton(),     # y
            "kill":             StickyButton(),     # window
            "reverse":          StickyButton(),     # menu
        }

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
            "linear_y":         joy_msg.axes[0],                # left_stick_x
            "linear_x":         joy_msg.axes[1],                # left_stick_y
            # "":               joy_msg.buttons[9],             # left_stick_press
            # Right stick
            "angular_z":        joy_msg.axes[3],                # right_stick_x
            "angular_y":        joy_msg.axes[4],                # right_stick_y
            # "":               joy_msg.buttons[10],            # right_stick_press
            # Triggers
            "neg_linear_z":     joy_msg.axes[2],                # left_trigger
            "pos_linear_z":     joy_msg.axes[5],                # right_trigger
            # Dpad
            # "":               joy_msg.axes[7],                # dpad up/dowm (used for tilty thing)
            # "":               joy_msg.axes[6]                 # dpad_left/right  (used for spinny thing)
            # Buttons
            "back_claw":        joy_msg.buttons[0], # a
            "bambi_mode":       joy_msg.buttons[1], # b
            "toggle_claw":      joy_msg.buttons[2], # x
            "articulate_claw":  joy_msg.buttons[3], # y
            "pos_angular_x":    joy_msg.buttons[4], # left_bumper
            "neg_angular_x":    joy_msg.buttons[5], # right_bumper
            "kill":             joy_msg.buttons[6], # window
            "reverse":          joy_msg.buttons[7], # menu
            "reset":            joy_msg.buttons[8], # xbox
        }

        # Create twist message
        twist_msg = Twist()

        # Kill motors with kill button
        if not (kill:=self.buttons["kill"].check_state(controller["kill"])):
            twist_msg.linear.x  = self.throttle_curve(controller["linear_x"])     # forwards
            twist_msg.linear.y  = -self.throttle_curve(-controller["linear_y"])   # sideways
            twist_msg.linear.z  = self.throttle_curve(((controller["neg_linear_z"] - controller["pos_linear_z"]) / 2))    # depth
            twist_msg.angular.x = -self.throttle_curve((controller["pos_angular_x"] - controller["neg_angular_x"]) * 0.5) # roll (const +/- 0.5 thrust)
            twist_msg.angular.y = self.throttle_curve(controller["angular_y"])    # pitch
            twist_msg.angular.z = self.throttle_curve(controller["angular_z"])    # yaw
        else:
            twist_msg.linear.x  = 0.0
            twist_msg.linear.y  = 0.0
            twist_msg.linear.z  = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0

        # Bambi mode cuts all twist values in half for more precise movements
        if bambi_state := self.buttons["bambi_mode"].check_state(controller["bambi_mode"]):
            twist_msg.linear.x  /= 2
            twist_msg.linear.y  /= 2
            twist_msg.linear.z  /= 2
            twist_msg.angular.x /= 2
            twist_msg.angular.y /= 2
            twist_msg.angular.z /= 2
        
        if reverse := self.buttons["reverse"].check_state(controller["reverse"]):
            twist_msg.linear.x  *= -1
            twist_msg.linear.y  *= -1
            twist_msg.angular.x *= -1
            twist_msg.angular.y *= -1

        # Publish twist message
        self.twist_pub.publish(twist_msg)

        # If the x-box button is pressed, all settings get reset to default configurations
        if controller["reset"]:
            self.buttons["bambi_mode"].reset()
            self.buttons["reverse"].reset()

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PilotInput())
    rclpy.shutdown()


if __name__ == "main":
    main(sys.argv)
