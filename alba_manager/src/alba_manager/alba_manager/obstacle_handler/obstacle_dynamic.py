#!/usr/bin/env python3
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import time
import numpy as np


class DynamicObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_avoidance')
        self.get_logger().info('Dynamic Obstacle Handler Node Started...')
        self.declare_parameter('obstacle_distance_threshold', 0.30)
        self.obstacle_distance_threshold = self.get_parameter('obstacle_distance_threshold').value

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.current_goal = None
        self.obstacle_detected = False

        self.filter_window = 3
        self.recent_ranges = {}  # 각도(deg) → deque

        # -180° ~ +180° 기준으로 해석된 135°~225° 영역: 실제로는 [135~175] + [-175~-135]
        self.target_degrees = list(range(135, 180, 10)) + list(range(-175, -130, 10))  # 총 10개 각도

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info('New goal received')
        self.current_goal = msg

    def scan_callback(self, msg: LaserScan):
        angle_to_median = {}
        filtered_values = []

        for deg in self.target_degrees:
            angle_rad = math.radians(deg)
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)

            if 0 <= index < len(msg.ranges):
                r = msg.ranges[index]
                if not math.isinf(r) and not math.isnan(r) and r > 0.01:
                    if deg not in self.recent_ranges:
                        self.recent_ranges[deg] = deque(maxlen=self.filter_window)
                    self.recent_ranges[deg].append(r)

                if deg in self.recent_ranges and len(self.recent_ranges[deg]) == self.filter_window:
                    median = np.median(self.recent_ranges[deg])
                    angle_to_median[deg] = median
                    filtered_values.append(median)

        # 디버그 출력
        self.get_logger().info("\n=== Filtered median distances ===")
        for deg in sorted(angle_to_median):
            dist = angle_to_median[deg]
            self.get_logger().info(f"{deg:+4}° → {dist:.3f} m")

        # 장애물 판단
        if any(dist < self.obstacle_distance_threshold for dist in filtered_values):
            if not self.obstacle_detected:
                self.obstacle_detected = True
                warn_angles = [deg for deg, dist in angle_to_median.items() if dist < self.obstacle_distance_threshold]
                self.get_logger().warn(f'⚠ Obstacle detected at angles: {warn_angles}. Stop and Waiting...')
                self.stop_robot()
                self.stop_duration()
        else:
            self.obstacle_detected = False

        self.get_logger().info(f"Obstacle detedted: {self.obstacle_detected}\n\n")

    def stop_robot(self):
        self.get_logger().info('Stop Robot...')
        twist = Twist()
        self.cmd_pub.publish(twist)

    def stop_duration(self, duration_sec=2):
        self.get_logger().info(f'Waiting for {duration_sec}sec...')
        twist = Twist()

        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=duration_sec)

        while self.get_clock().now() - start_time < duration:
            self.cmd_pub.publish(twist)

        self.stop_robot()
        # 필터 초기화 → 장애물 거리 초기화
        self.recent_ranges.clear()
        self.get_logger().info('Filter cleared after waiting.')

def run_dynamic_obstacle_handler():
    rclpy.init()
    node = DynamicObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    try:
        rclpy.init(args=args)
    except Exception as e:
        print(f"[❌] rclpy.init() failed: {e}")
        return

    node = DynamicObstacleAvoidance()  
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[⚠️] KeyboardInterrupt received. Exiting cleanly.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[✅] Node shutdown complete.")


if __name__ == '__main__':
    main()