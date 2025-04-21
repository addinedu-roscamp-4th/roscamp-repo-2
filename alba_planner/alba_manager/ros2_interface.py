import rclpy
from rclpy.node import Node
from robodine_msgs.msg import RobotStatus
import json
import socket
from datetime import datetime

class AlbaRos2TCPBridge(Node):
    def __init__(self):
        super().__init__('alba_ros2_tcp_bridge')
        
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10
        )
        
        self.tcp_host = "127.0.0.1"
        self.tcp_port = 8001

    def status_callback(self, msg: RobotStatus):
        data = {
            "id": msg.robot_id,
            "status": msg.status,
            "location": {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z
            },
            "battery_level": msg.battery_level,
            "timestamp": datetime.now().isoformat()
        }

        self.send_tcp_data(data)

    def send_tcp_data(self, data):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((self.tcp_host, self.tcp_port))
                sock.sendall(json.dumps(data).encode('utf-8'))
                response = sock.recv(1024).decode('utf-8')
                self.get_logger().info(f"서버 응답: {response}")
        except Exception as e:
            self.get_logger().error(f"TCP 연결 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AlbaRos2TCPBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
