# File: alba_planner/alba_manager/ros2_interface.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from datetime import datetime
import json
from src.alba_manager.alba_manager.tcp_client import AlbaTCPClient

class AlbaRos2TCPBridge(Node):
    def __init__(self):
        super().__init__('alba_ros2_tcp_bridge')
        # TCP client for sending data to RoboDine Service
        self.tcp_client = AlbaTCPClient(host='127.0.0.1', port=8001)

        # Subscribe to ROS2 topic 'robot_pose'
        self.get_logger().info('🌟 AlbaRos2TCPBridge 노드 초기화')
        self.subscription = self.create_subscription(
            PoseStamped,
            'robot_pose',
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('🛰️ 구독 중인 토픽: robot_pose')

    def pose_callback(self, msg: PoseStamped):
        # Log reception
        self.get_logger().info(f'📡 Pose 데이터 수신: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}')

        # Prepare data payload
        data = {
            'id': 'robot1',  # 실제 ID 로직으로 대체할 것
            'status': 'moving',
            'location': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'battery_level': 90,
            'timestamp': datetime.utcnow().isoformat()
        }

        # Send via TCP
        try:
            response = self.tcp_client.send_data(data)
            self.get_logger().info(f'✅ 서버 응답: {response}')
        except Exception as e:
            self.get_logger().error(f'❌ TCP 전송 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AlbaRos2TCPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('중지 요청으로 노드를 종료합니다...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
