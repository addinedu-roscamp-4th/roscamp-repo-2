import rclpy
from rclpy.node import Node
from mycobot_interfaces.msg import MycobotAngles, MycobotCoords
import time

class M5TeleopCommander(Node):
    def __init__(self):
        super().__init__('m5_teleop_commander')

        self.declare_parameter('ns', 'robot48')  # 로봇에서 사용하는 네임스페이스
        ns = self.get_parameter('ns').get_parameter_value().string_value

        self.get_logger().info(f"🔵 연결할 네임스페이스: {ns}")

        # 구독자 (로봇이 보내는 실제 상태를 받음)
        self.create_subscription(MycobotAngles, f'{ns}/angles_real', self.angles_real_callback, 10)
        self.create_subscription(MycobotCoords, f'{ns}/coords_real', self.coords_real_callback, 10)

        # 퍼블리셔 (로봇에게 명령 보냄)
        self.angles_pub = self.create_publisher(MycobotAngles, f'{ns}/angles_targ', 10)
        self.coords_pub = self.create_publisher(MycobotCoords, f'{ns}/coords_targ', 10)

        # 테스트용: 일정 주기로 명령 보내기 (옵션)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0

    def angles_real_callback(self, msg):
        angles = [msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6]
        self.get_logger().info(f"🟢 현재 관절 상태 수신: {angles}")

    def coords_real_callback(self, msg):
        coords = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]
        self.get_logger().info(f"🟢 현재 좌표 상태 수신: {coords}")

    def timer_callback(self):
        # 예시: 주기적으로 명령을 보낸다
        angles_msg = MycobotAngles()
        coords_msg = MycobotCoords()

        # 단순히 카운터를 활용한 테스트용 명령
        delta = 10.0 * (1 if (self.counter % 2 == 0) else -1)
        
        angles_msg.joint_1 = delta
        angles_msg.joint_2 = delta
        angles_msg.joint_3 = delta
        angles_msg.joint_4 = delta
        angles_msg.joint_5 = delta
        angles_msg.joint_6 = delta

        # coords_msg.x = 200.0 + delta
        # coords_msg.y = 0.0
        # coords_msg.z = 250.0
        # coords_msg.rx = 0.0
        # coords_msg.ry = 0.0
        # coords_msg.rz = 0.0

        self.get_logger().info(f"🟡 관절 명령 발행: {[angles_msg.joint_1, angles_msg.joint_2, angles_msg.joint_3, angles_msg.joint_4, angles_msg.joint_5, angles_msg.joint_6]}")
        # self.get_logger().info(f"🟡 좌표 명령 발행: {[coords_msg.x, coords_msg.y, coords_msg.z, coords_msg.rx, coords_msg.ry, coords_msg.rz]}")

        self.angles_pub.publish(angles_msg)
        # self.coords_pub.publish(coords_msg)

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = M5TeleopCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()