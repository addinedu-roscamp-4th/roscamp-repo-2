import rclpy
from rclpy.node import Node
from robodine_msgs.msg import RobotStatus
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time

class RobotStatusPublisher(Node):

    def __init__(self):
        super().__init__('robot_status_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = RobotStatus()

        msg.id = 1                            
        msg.status = "IDLE"
        msg.type = "PINKY1"

        # position 필드 초기화
        msg.position = Pose()

        # position 내의 position(Point) 값
        msg.position.position.x = 1.0
        msg.position.position.y = 2.0
        msg.position.position.z = 0.0

        # position 내의 orientation(Quaternion) 값 설정 (기본값으로도 가능)
        msg.position.orientation.x = 0.0
        msg.position.orientation.y = 0.0
        msg.position.orientation.z = 0.0
        msg.position.orientation.w = 1.0

        msg.battery_level = 90.0

        now = self.get_clock().now().to_msg()
        msg.timestamp = now

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: id={msg.id}, type={msg.type}, status={msg.status}')

def main(args=None):
    rclpy.init(args=args)
    robot_status_publisher = RobotStatusPublisher()
    rclpy.spin(robot_status_publisher)
    robot_status_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
