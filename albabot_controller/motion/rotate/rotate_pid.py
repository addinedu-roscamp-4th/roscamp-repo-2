#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import math
import tf_transformations
import sys
import time
import os
from datetime import datetime

class SimplePID:
    def __init__(self, P=1.5, I=0.0, D=0.1, output_limit=0.1):
        self.P = P
        self.I = I
        self.D = D
        self.output_limit = output_limit
        self.prev_error = 0.0
        self.integral = 0.0
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.P * error + self.I * self.integral + self.D * derivative
        output = max(min(output, self.output_limit), -self.output_limit)
        self.prev_error = error
        return output
    
class RotateToYawPID(Node):
    def __init__(self, target_yaw, mode=None, fix_one_side=False):
        super().__init__('rotate_to_yaw_pid')
        self.pid = SimplePID(P=0.2, I=0.0, D=0.1, output_limit=0.01)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_yaw = target_yaw  # 목표 yaw 설정
        self.reached = False
        self.tolerance_rad = math.radians(2.0)  
        self.mode = mode                # 0 또는 1
        self.fix_one_side = fix_one_side  # True: 회전 방향 고정
    def pose_callback(self, msg):
        q = msg.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        error = self.normalize_angle(self.target_yaw - yaw)
        twist = Twist()
        if abs(error) < self.tolerance_rad:
            twist.angular.z = 0.0
            self.reached = True
            self.get_logger().info(f"✅ 목표 Yaw {math.degrees(self.target_yaw):.2f}° 도달. 정지.")
        else:
            output = self.pid.compute(error)
            if self.fix_one_side and self.mode is not None:
                twist.linear.x = abs(output)
                if self.mode == 0:
                    # 오른쪽 고정: 시계방향으로만 회전
                    twist.angular.z = -min(abs(output), 0.01)
                    print(f"rotation speed = {twist.angular.z}") 
                elif self.mode == 1:
                    # 왼쪽 고정: 반시계방향으로만 회전
                    twist.angular.z = min(abs(output), 0.01) 
                    print(f"rotation speed = {twist.angular.z}") 
            else:
                # 일반 PID 회전(방향 상관 없이 가까운 쪽으로 회전))
                max_speed = 0.01  # 최대 회전 속도(rad/s)
                limited_output = max(min(output, max_speed), -max_speed)
                twist.angular.z = limited_output
                print(f"rotation speed = {twist.angular.z}") 
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f"[현재 Yaw] {math.degrees(yaw):.2f}° | [목표] {math.degrees(self.target_yaw):.2f}° | "
            f"[오차] {math.degrees(error):.2f}° | [PID출력] {twist.angular.z:.3f} rad/s"
        )
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
        
    