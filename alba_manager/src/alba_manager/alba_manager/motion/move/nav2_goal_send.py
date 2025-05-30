#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import tf_transformations

# 목표 goal에 도달 -> reached=1 else 0
# 목표 goal nav2에 전달 -> send_done = True else False

class Nav2GoalSender(Node):
    def __init__(self, x: float, y: float, yaw_deg: float):
        super().__init__('nav2_goal_sender')
        self.reached = None
        self.send_done = False
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.send_goal(x, y, yaw_deg)

    def send_goal(self, x, y, yaw_deg):
        self.get_logger().info(f'🎯 목표 좌표 수신: x={x}, y={y}, yaw={yaw_deg} deg')

        while not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('⏳ Nav2 action 서버를 기다리는 중...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # 위치 설정
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # 방향 설정 (Euler → Quaternion)
        yaw_rad = math.radians(yaw_deg)
        q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ Goal이 거부됨')
            self.reached = 0
            self.send_done = True
            return
        self.get_logger().info('✅ Goal이 수락되었습니다.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('🏁 Goal에 도달완료.')
        self.reached = 1
        self.send_done = True


def send_goal_and_wait(x, y, yaw_deg) -> int:
    # rclpy.init()
    node = Nav2GoalSender(x, y, yaw_deg)
    while rclpy.ok() and node.reached is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = node.reached
    node.destroy_node()
    #rclpy.shutdown()
    return result



def main(args=None):
    rclpy.init(args=args)

    # 좌표 인자 (예: x=1.0, y=0.5, yaw=90도)
    x = 0.98590
    y = -0.00164
    yaw_deg = 0.0

    node = Nav2GoalSender(x, y, yaw_deg)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
