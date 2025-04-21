# File: test_ros2_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random

class TestPosePublisher(Node):
    def __init__(self):
        super().__init__('test_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = random.uniform(0.0, 10.0)
        msg.pose.position.y = random.uniform(0.0, 10.0)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)
        self.get_logger().info(f"발행된 데이터: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TestPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
