#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DistanceMover(Node):
    def __init__(self, speed, distance):
        super().__init__('distance_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 이동할 시간 계산
        duration = distance / speed

        # 이동 명령 메시지 생성
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        self.get_logger().info(f'이동 시작: {distance:.2f}m를 {speed:.2f} m/s로 이동합니다 ({duration:.2f}초)')

        # 이동 명령 발행
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time < duration):
            self.publisher.publish(twist)
            time.sleep(0.05)  # 20 Hz

        # 정지 명령
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('🛑 이동 완료 및 정지')

def main():
    rclpy.init()
    speed = 0.1       # m/s
    distance = 0.05    # m
    node = DistanceMover(speed, distance)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
