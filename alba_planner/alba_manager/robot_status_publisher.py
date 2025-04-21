import rclpy
from rclpy.node import Node
from robodine_msgs.msg import RobotStatus
from geometry_msgs.msg import Pose
import random

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.robot_id = "robot1"
        msg.pose.position.x = random.uniform(0.0, 5.0)
        msg.pose.position.y = random.uniform(0.0, 5.0)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.battery_level = random.uniform(20.0, 100.0)
        msg.status = "moving"

        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.robot_id}, position: {msg.pose.position.x}, battery: {msg.battery_level}")

def main(args=None):
    rclpy.init(args=args)
    publisher_node = RobotStatusPublisher()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
