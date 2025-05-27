#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class DShapeMotion(Node):
    def __init__(self):
        super().__init__('d_shape_motion')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.1   # m/s
        self.angular_speed_deg = 30.0  # deg/s
        self.angular_speed = math.radians(self.angular_speed_deg)

        # 시나리오 실행
        self.run_sequence()

    def move_forward(self, distance):
        duration = distance / self.linear_speed
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0

        self.get_logger().info(f'🚗 전진 {distance:.2f}m ({duration:.2f}s)')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.stop_robot()

    def rotate_in_place(self, angle_deg):
        direction = -1 if angle_deg < 0 else 1
        angle_rad = abs(math.radians(angle_deg))
        duration = angle_rad / self.angular_speed

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = direction * self.angular_speed

        self.get_logger().info(f'↩ 회전 {angle_deg}° ({duration:.2f}s)')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        time.sleep(0.2)

    def run_sequence(self):
        self.move_forward(1.3)       # 1
        self.rotate_in_place(-90)    # 2
        self.move_forward(0.1)       # 3
        self.rotate_in_place(-90)    # 4
        self.move_forward(1.3)       # 5
        self.rotate_in_place(-90)    # 6
        self.move_forward(0.1)       # 7
        self.rotate_in_place(-90)    # 8
        self.get_logger().info("✅ 'ㄷ'자 동작 완료")

def main(args=None):
    rclpy.init(args=args)
    node = DShapeMotion()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
