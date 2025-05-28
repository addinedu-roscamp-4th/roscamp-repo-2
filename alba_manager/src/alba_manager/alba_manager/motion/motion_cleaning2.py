#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
from motion.move.nav2_goal_send import send_goal_and_wait


class CleaningMotion(Node):
    def __init__(self, goal1, goal2, yaw1, yaw2):
        super().__init__('cleaning')


    def move_forward(self):
        self.get_logger().info("start cleaning.")

    def rotate_in_place(self, angle_deg):
        self.get_logger().info("rotate start")


    def move_return(self):
        self.get_logger().info("start ")



def main(args=None):
    rclpy.init(args=args)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
