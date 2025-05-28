#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
import time
from rotate.rotate_pid import RotateToYawPID
from move.nav2_goal_send import send_goal_and_wait
from move.move_straight import DistanceMover  # 10cm 전진

class Cleaning(Node):
    def __init__(self, mode=1):
        super().__init__('cleaning')
        if mode == 1:
            # goal 1 → 90도 회전 → 전진 → 90도 회전 → goal 2 → 180도 회전
            self.goal_1_x, self.goal_1_y = (1.202, 0.013)
            self.goal_2_x, self.goal_2_y = (-0.061, -0.122)
            self.rotate_deg_1 = -90.0
            self.rotate_deg_2 = -90.0
            self.final_rotate_deg = -180.0
            self.distance = 0.05
        else:
            self.get_logger().error("❌ 지원하지 않는 mode입니다.")
            return

        self.run_sequence()

    def run_sequence(self):
        # 1. 첫 번째 Nav2 이동
        self.get_logger().info("🚗 [1] 첫 번째 지점으로 이동 중...")
        result = send_goal_and_wait(self.goal_1_x, self.goal_1_y, 0.0)
        if result != 1:
            self.get_logger().error("❌ 첫 번째 이동 실패")
            return

        # 2. 첫 번째 회전
        self.get_logger().info("🔁 [2] -90도 회전 중...")
        self.rotate(self.rotate_deg_1)

        # 3. 직진
        self.get_logger().info("🏃 [3] 10cm 직진 중...")
        self.move_forward(self.distance)

        # 4. 두 번째 회전
        self.get_logger().info("🔁 [4] -90도 회전 중...")
        self.rotate(self.rotate_deg_2)

        # 5. 두 번째 Nav2 이동
        self.get_logger().info("🚗 [5] 두 번째 지점으로 이동 중...")
        result = send_goal_and_wait(self.goal_2_x, self.goal_2_y, 180.0)
        if result != 1:
            self.get_logger().error("❌ 두 번째 이동 실패")
            return

        # 6. 마지막 회전 (정렬)
        self.get_logger().info("🔁 [6] 최종 방향 정렬 중...")
        self.rotate(self.final_rotate_deg)

        self.get_logger().info("✅ Cleaning 시나리오 완료!")

    def rotate(self, yaw_deg):
        node = RotateToYawPID(math.radians(yaw_deg))
        while rclpy.ok() and not node.reached:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()

    def move_forward(self, distance):
        mover = DistanceMover(speed=0.05, distance=distance)
        rclpy.spin_once(mover, timeout_sec=0.1)
        mover.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Cleaning(mode=1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
