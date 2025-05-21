import rclpy
from rclpy.node import Node
from mycobot_interfaces.msg import MycobotAngles, MycobotCoords
from std_msgs.msg import String
from queue import Queue
import socket
import json
import threading
import time
import requests

SERVER_IP = '192.168.0.156'
SERVER_PORT = 8001


class TCPInterface(Node):
    def __init__(self):
        super().__init__('pose_broadcaster')
        self.robot_list = ['robot48', 'robotb4']
        self.latest_angles = {}
        self.latest_coords = {}
        self.menu_items = []
        self.menu_queue = Queue()
        self.cookbot_state = "IDLE"

        # 각 로봇 토픽 설정
        for ns in self.robot_list:
            self.create_subscription(MycobotAngles, f'{ns}/angles_real', self.make_angles_callback(ns), 10)
            self.create_subscription(MycobotCoords, f'{ns}/coords_real', self.make_coords_callback(ns), 10)
            
        self.create_subscription(String,'/cook_state', self.cook_state_callback, 10)
        self.menu_command_pub = self.create_publisher(String, '/menu_item', 10)
        # self.create_timer(0.1, self.publish_next_menu_item)  # 0.1초마다 처리
        # 주기적으로 TCP 전송 (1초 간격)
        self.timer = self.create_timer(1.0, lambda: self.send_data_to_server(self.cookbot_state))
        self.create_timer(5.0, self.get_menu)

        # TCP 수신 스레드 시작
        # threading.Thread(target=self.tcp_listener, daemon=True).start()

    def make_angles_callback(self, ns):
        def callback(msg):
            self.latest_angles[ns] = [msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6]
        return callback

    def make_coords_callback(self, ns):
        def callback(msg):
            self.latest_coords[ns] = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]
        return callback
    
    def cook_state_callback(self, msg):
        self.cookbot_state = msg.data

    def send_payload(self, host, port, payload: dict):
        raw = json.dumps(payload).encode('utf-8')
        # 1) 4바이트 big-endian 헤더
        header = len(raw).to_bytes(4, byteorder='big')
        # 2) 헤더 + 바디를 한 번에 전송
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((host, port))
            sock.sendall(header + raw)
            resp = sock.recv(1024)
            # 헤더(4바이트) 제거 후 JSON 디코드
            if len(resp) >= 4:
                json_bytes = resp[4:]  # 헤더 제거
                try:
                    data = json.loads(json_bytes.decode('utf-8'))
                    # print("🟢 서버 응답 (파싱 성공):", data)

                    # 상태 확인용 로그
                    if data.get("status") != "success":
                        self.get_logger().warn(f"❌ 서버 응답 에러: {data.get('message')}")
                except Exception as e:
                    print("⚠️ 서버 응답 디코드 실패:", e)
                    # print("📦 응답(raw):", json_bytes)
            else:
                print("⚠️ 응답이 너무 짧습니다:", resp)

    def send_data_to_server(self, robotstatus):
        ROBOT_ID_MAP = {
            'robot48': 5,
            'robotb4': 4
        }
        # robotstatus = "IDLE","SETTING", "COOKING", "PICKUP"
        try:
            # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            #     s.connect((SERVER_IP, SERVER_PORT))
            # send_payload에서 연결함

            # 로봇별 메시지를 순차적으로 전송
            for i, ns in enumerate(self.robot_list):
                angles = self.latest_angles.get(ns)
                if not angles or len(angles) < 6 or any(a is None for a in angles):
                    self.get_logger().warn(f"❗ {ns}의 각도 데이터가 비정상적임: {angles}. 기본값으로 대체.")
                    angles = [0] * 6

                coords = self.latest_coords.get(ns)
                if not coords or len(coords) < 6 or any(c is None for c in coords):
                    self.get_logger().warn(f"❗ {ns}의 위치 데이터가 비정상적임: {coords}. 기본값으로 대체.")
                    coords = [0] * 6

                payload = {
                    "msg_type": "Cookbot",
                    "robot_id": ROBOT_ID_MAP.get(ns, -1),  # robot_list 순서대로 1, 2, ...
                    "status": robotstatus,
                    "angle_1": angles[0],
                    "angle_2": angles[1],
                    "angle_3": angles[2],
                    "angle_4": angles[3],
                    "angle_5": angles[4], 
                    "angle_6": angles[5],
                    "endpoint_x": coords[0],
                    "endpoint_y": coords[1],
                    "endpoint_z": coords[2],
                    "endpoint_roll": coords[3],
                    "endpoint_pitch": coords[4],
                    "endpoint_yaw": coords[5],
                }
                self.send_payload(SERVER_IP, SERVER_PORT, payload)
                # self.get_logger().info(f"[📤 전송됨] {json_data}")

        except Exception as e:
            self.get_logger().warn(f"[⚠️ TCP 전송 오류] {e}")


    def publish_next_menu_item(self):
        if not self.menu_queue.empty():
            command = self.menu_queue.get()
            msg = String()
            msg.data = command
            self.menu_command_pub.publish(msg)
            # self.get_logger().info(f"📢 ROS 토픽 전송: {command}")

    def get_menu(self):
        try:
            # self.get_logger().info(f"📦 요청요청: ")
            url = "http://192.168.0.156:8000/api/orders/todo_order"
            response = requests.get(url)
            if response.status_code == 200:
                menu_data = response.json()
                {'order_id': 1, 'item_name': 'salad'}
                self.get_logger().info(f"📦 수신된 메뉴: {menu_data}")
                self.menu_items = menu_data.get('item_name', [])
                msg = String()
                msg.data = self.menu_items
                self.menu_command_pub.publish(msg)
            else:
                self.get_logger().warn(f"❌ 메뉴 요청 실패 - 상태 코드 {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"⚠️ REST API 통신 실패: {e}")

    def tcp_listener(self):
        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('0.0.0.0', 8002))  # 이 포트로 명령 수신
                    s.listen(1)
                    # self.get_logger().info("📡 명령 수신 대기 (TCP 8002)")

                    conn, addr = s.accept()
                    with conn:
                        # self.get_logger().info(f"🔗 연결됨: {addr}")
                        while True:
                            data = conn.recv(1024)
                            if not data:
                                break
                            command = data.decode('utf-8').strip()

                            # 리스트에 저장
                            self.menu_queue.put(command)
                            # self.get_logger().info(f"📥 명령 수신: {command}")  

                            self.publish_next_menu_item() 

            except Exception as e:
                self.get_logger().warn(f"[❌ TCP 수신 오류] {e}")
                time.sleep(2)  # 재시도 지연

def main(args=None):
    rclpy.init(args=args)
    node = TCPInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
