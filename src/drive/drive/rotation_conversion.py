import math

import rclpy
from rclpy.node import Node

import os

from geometry_msgs.msg import Point
from drive_interface.msg import EncoderVals, RotationGoals


class RotationConversionNode(Node):

    def __init__(self):
        super().__init__('rot_conv')
        self.subscription = self.create_subscription(Point, '/relative_cone', self.listener_callback, 10)
        self.goal_pub = self.create_publisher(RotationGoals, '/rotation_goals', 10)
        self.subscription  # prevent unused variable warning
        self.b = 0.225
        self.R = 0.05

    def listener_callback(self, msg):
        """
        Callback finds polar coordinates of location from Cartesian in ground plane, then finds number of rotations for each motor
        """
        ## length in meters
        ## angle in radians
        distance = math.sqrt(msg.x ** 2 + msg.y ** 2)
        angle = math.atan(msg.y / msg.x)

        des_forward_vel = 0.0 if distance < 0.245 else distance
        des_angular_vel = 0 if abs(angle) < 0.1 else angle

        des_forward_vel = min(0.75, des_forward_vel)
        # des_angular_vel = max(-0.5, min(0.5, des_angular_vel))

        left_angular_velocity = -(des_forward_vel - (self.b * des_angular_vel)/2)/self.R
        right_angular_velocity = -(des_forward_vel + (self.b * des_angular_vel)/2)/self.R

        goals_msg = RotationGoals()
        goals_msg.l_goal = left_angular_velocity
        goals_msg.r_goal = right_angular_velocity
        self.goal_pub.publish(goals_msg)
        
        


def main(args=None):
    print(os.getcwd())
    rclpy.init(args=args)

    rot_conv = RotationConversionNode()

    rclpy.spin(rot_conv)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rot_conv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
