#!/usr/bin/env python3
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import numpy as np
import atexit


class StaticObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('static_obstacle_avoidance')
        self.get_logger().info('Static Obstacle Handler Node Started...')
        self.declare_parameter('obstacle_distance_threshold', 0.10)
        self.obstacle_distance_threshold = self.get_parameter('obstacle_distance_threshold').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.current_goal = None
        self.obstacle_detected = False
        self.filter_window = 5
        self.recent_ranges = {}
        self.target_degrees = list(range(135, 180, 10)) + list(range(-175, -130, 10))

        # 종료 시 무조건 정지
        atexit.register(self._on_exit)

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

        self.get_logger().info("\n=== Filtered median distances ===")
        for deg in sorted(angle_to_median):
            dist = angle_to_median[deg]
            self.get_logger().info(f"{deg:+4}° → {dist:.3f} m")

        if any(dist < self.obstacle_distance_threshold for dist in filtered_values):
            if not self.obstacle_detected:
                self.obstacle_detected = True
                warn_angles = [deg for deg, dist in angle_to_median.items() if dist < self.obstacle_distance_threshold]
                self.get_logger().warn(f'⚠ Obstacle detected at angles: {warn_angles}. Stopping and backing up...')
                self.stop_robot()
                self.backup(warn_angles)
        else:
            self.obstacle_detected = False

        self.get_logger().info(f"Obstacle detedted: {self.obstacle_detected}\n\n")

    def stop_robot(self):
        twist = Twist()
        if rclpy.ok():  # ROS context가 살아 있을 때만 publish
            self.get_logger().info('Stop Robot...')
            self.cmd_pub.publish(twist)

    def _on_exit(self):
        try:
            self.stop_robot()
        except Exception:
            pass

    def backup(self, obstacle_angles=None, duration_sec=0.5):
        if obstacle_angles is None:
            obstacle_angles = []

        left_set = {-155, -145, -135}
        right_set = {135, 145, 155}
        left_hits = [deg for deg in obstacle_angles if deg in left_set]
        right_hits = [deg for deg in obstacle_angles if deg in right_set]

        rotate_twist = Twist()
        need_recovery = False

        if len(left_hits) >= 1 and len(right_hits) == 0:
            self.get_logger().info('🌀 Rotate counter-clockwise before backing up')
            rotate_twist.angular.z = 0.5
            need_recovery = True
        elif len(right_hits) >= 1 and len(left_hits) == 0:
            self.get_logger().info('🌀 Rotate clockwise before backing up')
            rotate_twist.angular.z = -0.5
            need_recovery = True
        else:
            self.get_logger().info('No specific rotation needed')
            rotate_twist = None

        if rotate_twist is not None:
            start_time = self.get_clock().now()
            rotate_duration = rclpy.duration.Duration(seconds=0.3)
            while self.get_clock().now() - start_time < rotate_duration:
                self.cmd_pub.publish(rotate_twist)
            self.stop_robot()

        self.get_logger().info('⬅ Moving backward...')
        twist = Twist()
        twist.linear.x = -0.05

        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=duration_sec)
        while self.get_clock().now() - start_time < duration:
            self.cmd_pub.publish(twist)
        self.stop_robot()

        if need_recovery:
            self.get_logger().info('↩ Rotating back to original heading...')
            recovery_twist = Twist()
            recovery_twist.angular.z = -rotate_twist.angular.z
            start_time = self.get_clock().now()
            while self.get_clock().now() - start_time < rotate_duration:
                self.cmd_pub.publish(recovery_twist)
            self.stop_robot()

        self.recent_ranges.clear()
        self.get_logger().info('Filter cleared after backup.')

def main(args=None):
    rclpy.init(args=args)
    node = StaticObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[⚠️] Ctrl+C received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[✅] Node shutdown complete.")

if __name__ == '__main__':
    main()
