# ros2_interface.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tcp_client import AlbaTCPClient
import json
from datetime import datetime

class ROS2Interface(Node):
    def __init__(self):
        super().__init__('alba_ros2_interface')
        self.subscription = self.create_subscription(String, 'robot_state', self.listener_callback, 10)
        self.tcp_client = AlbaTCPClient()
        self.tcp_client.connect()

    def listener_callback(self, msg):
        ros_data = json.loads(msg.data)
        ros_data["timestamp"] = datetime.utcnow().isoformat()
        self.tcp_client.send_data(ros_data)
        self.get_logger().info(f'Sent ROS2 data to RoboDine Service: {ros_data}')

def main(args=None):
    rclpy.init(args=args)
    node = ROS2Interface()
    rclpy.spin(node)
    node.tcp_client.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
