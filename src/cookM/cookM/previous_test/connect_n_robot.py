import rclpy
from rclpy.node import Node
from mycobot_interfaces.msg import MycobotAngles, MycobotCoords

class M5TeleopCommander(Node):
    def __init__(self):
        super().__init__('m5_teleop_commander')

        # 여러 로봇의 네임스페이스를 리스트로 정의
        self.robot_list = ['robot48', 'robotb4']
        self.get_logger().info(f"🔵 연결할 로봇들: {self.robot_list}")

        # 로봇별 퍼블리셔, 서브스크라이버 저장
        self.angles_pubs = {}
        self.coords_pubs = {}

        # 타이머용 카운터
        self.counter = 0

        # 각 로봇에 대해 구독자 및 퍼블리셔 설정
        for ns in self.robot_list:
            self.create_subscription(MycobotAngles, f'{ns}/angles_real', self.make_angles_callback(ns), 10)
            self.create_subscription(MycobotCoords, f'{ns}/coords_real', self.make_coords_callback(ns), 10)

            self.angles_pubs[ns] = self.create_publisher(MycobotAngles, f'{ns}/angles_targ', 10)
            self.coords_pubs[ns] = self.create_publisher(MycobotCoords, f'{ns}/coords_targ', 10)

        # 타이머 등록
        self.create_timer(2.0, self.timer_callback)

    def make_angles_callback(self, ns):
        def callback(msg):
            angles = [msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6]
            self.get_logger().info(f"🟢 [{ns}] 현재 관절 상태 수신: {angles}")
        return callback

    def make_coords_callback(self, ns):
        def callback(msg):
            coords = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]
            self.get_logger().info(f"🟢 [{ns}] 현재 좌표 상태 수신: {coords}")
        return callback

    def timer_callback(self):
        # delta를 토글
        delta = 10.0 * (1 if (self.counter % 2 == 0) else -1)

        for ns in self.robot_list:
            angles_msg = MycobotAngles()
            angles_msg.joint_1 = delta
            angles_msg.joint_2 = delta
            angles_msg.joint_3 = delta
            angles_msg.joint_4 = delta
            angles_msg.joint_5 = delta
            angles_msg.joint_6 = delta

            self.get_logger().info(f"🟡 [{ns}] 관절 명령 발행: "
                                   f"[{angles_msg.joint_1}, {angles_msg.joint_2}, {angles_msg.joint_3}, "
                                   f"{angles_msg.joint_4}, {angles_msg.joint_5}, {angles_msg.joint_6}]")
            self.angles_pubs[ns].publish(angles_msg)

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = M5TeleopCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
