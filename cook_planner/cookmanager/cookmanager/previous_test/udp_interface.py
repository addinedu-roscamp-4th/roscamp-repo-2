import rclpy
from rclpy.node import Node
import socket


class UDPInterface(Node):
    def __init__(self):
        super().__init__('udp_interface')

        self.client_addr = ('192.168.0.148', 9001)
        self.pb_addr = ('127.0.0.1', 9100)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 9000))
        self.sock.settimeout(0.01)

        self.latest_signal = None  # PoseBroadcaster → client
        self.latest_client_pose = None  # client → PoseBroadcaster

        # self.get_logger().info("📡 수신 대기 (UDP 9000번 포트)")
        # self.get_logger().info(f"📤 클라이언트 → {self.client_addr}, PoseBroadcaster → {self.pb_addr}")

        self.timer_recv = self.create_timer(0.05, self.interface_udp)        # 모든 데이터 수신
        self.timer_to_client = self.create_timer(0.1, self.forward_to_client)  # 0.1초마다 client로 전송
        self.timer_to_pb = self.create_timer(1.0, self.forward_to_pb)          # 1초마다 broadcaster로 전송

    def interface_udp(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            decoded = data.decode().strip()
            tokens = decoded.split(',')

            # 📍송신자 구분
            if addr == self.pb_addr:
                # PoseBroadcaster → UDPinterface
                if len(tokens) == 1 and tokens[0].isdigit():
                    self.latest_signal = int(tokens[0])
                else:
                    self.get_logger().warn(f"⚠️ PoseBroadcaster 잘못된 메시지: {decoded}")

            elif addr == self.client_addr:
                # client → UDPinterface
                if len(tokens) == 7:
                    robot_name = tokens[0]
                    pose = list(map(float, tokens[1:]))
                    self.latest_client_pose = (robot_name, pose)
                else:
                    self.get_logger().warn(f"⚠️ client 잘못된 메시지: {decoded}")

            else:
                self.get_logger().warn(f"❓알 수 없는 송신자 {addr} → {decoded}")

        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().warn(f"❌ 수신 오류: {e}")


    def forward_to_client(self):
        if self.latest_signal is not None:
            try:
                msg = str(self.latest_signal).encode()
                self.sock.sendto(msg, self.client_addr)
                # self.get_logger().info(f"📤 클라이언트로 {self.latest_signal} 전송")
            except Exception as e:
                self.get_logger().warn(f"❌ 클라이언트 전송 실패: {e}")
            self.latest_signal = None  # 보낸 후 초기화

    def forward_to_pb(self):
        if self.latest_client_pose is not None:
            try:
                robot_name, pose = self.latest_client_pose
                msg = f"{robot_name}," + ",".join(map(str, pose))
                self.sock.sendto(msg.encode(), self.pb_addr)
                # self.get_logger().info(f"📤 PoseBroadcaster로 {robot_name} pose 전송")
            except Exception as e:
                self.get_logger().warn(f"❌ PoseBroadcaster 전송 실패: {e}")
            self.latest_client_pose = None  # 보낸 후 초기화


def main(args=None):
    rclpy.init(args=args)
    node = UDPInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
