import mediapipe as mp
import threading
import socket
import cv2
import numpy as np
import time
import json
import requests
from . import AlbaGPT_function
import base64
import rclpy
import os
import uuid

from langchain_core.callbacks.base import Callbacks
from langchain_core.caches import BaseCache
from langchain_openai.chat_models import ChatOpenAI
from albagpt_server.config import dynamic_object_list, static_object_list
from collections import deque, Counter
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String

memory_url = 'http://192.168.0.156:8000/api/chat/history?page=1&per_page=3' # AlbaBot의 응답을 저장해주는 url

ChatOpenAI.model_rebuild()

class AlbaGPTNode(Node):
    def __init__(self):
        super().__init__('albagpt_node')
        self.alba_ports = {'AlbaBot_1': 5000, 'AlbaBot_2': 5001, 'AlbaBot_3': 5002}
        self.declare_parameter('qos_depth', 10)
        self.latest_result_frame = {name: deque(maxlen=5) for name in self.alba_ports}
        self.frame_locks = {name: threading.Lock() for name in self.alba_ports} # 해당 쓰레드만 
        self.obstacle_info = {
            name: deque(maxlen=10) for name in self.alba_ports
        }
        self.fps_info = {
            name: {'cnt': 0, 'start_time': time.time(), 'fps': 0}
            for name in self.alba_ports
        }
        self.llm = ChatOpenAI(
            temperature=0.7,  # 창의성 (0.0 ~ 2.0)
            max_tokens=2048,  # 최대 토큰수
            model_name="gpt-4o",  # 모델명
        ) 

        # 스레드 시작
        for id_port in self.alba_ports.items():
            robot_id, port = id_port
            udp_server = threading.Thread(target=self.udp_server, args=(robot_id, port), daemon=True)
            udp_server.start()
        
        tcp_thread = threading.Thread(target=self.tcp_server, daemon=True)
        tcp_thread.start()
        self.get_logger().info("📡 TCP Server thread started.")

        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth)

        self.obstacle_information_publisher = self.create_publisher(
            String,
            'discriminated_obstacle',
            QOS_RKL10V)

        self.timer = self.create_timer(1.0, self.publish_obstacle_information)

    def udp_server(self, robot_id, port):
        """
        지정된 알바봇으로부터 영상 프레임을 받기 위한 UDP 서버를 열어주는 함수입니다.
        수신한 영상 프레임을 self.latest_frame으로 저장해주는 역할을 수행합니다.

        Returns :
            None
        """
        # UDP 서버 설정
        UDP_IP = "0.0.0.0"
        UDP_PORT = port
        BUFFER_SIZE = 65535

        # 미디어파이프 디텍터 모델 로드
        base_options = python.BaseOptions(model_asset_path='./contents/model/efficientdet_lite0.tflite')
        VisionRunningMode = mp.tasks.vision.RunningMode
        options = vision.ObjectDetectorOptions(base_options=base_options, score_threshold=0.4, max_results=-1, # 전부 반환
                                                running_mode=VisionRunningMode.IMAGE)
        detector = vision.ObjectDetector.create_from_options(options)

        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind((UDP_IP, UDP_PORT))
        self.get_logger().info(f"📡 {robot_id}:{port}의 카메라 수신 스레드 시작됨")

        while True:
            try:
                packet, addr = udp_socket.recvfrom(BUFFER_SIZE)
                np_data = np.frombuffer(packet, dtype=np.uint8)
                decoded_frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

                if decoded_frame is None:
                    continue
                else :
                    decoded_frame = cv2.resize(decoded_frame, (640, 480))
                    
                    info = self.fps_info[robot_id]
                    info['cnt'] += 1
                    elapsed = time.time() - info['start_time']
                    if elapsed >= 1:
                        info['fps'] = info['cnt']
                        info['cnt'] = 0
                        info['start_time'] = time.time()

                    # np.array -> Mediapipe Image
                    mediapipe_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=decoded_frame)

                    # 디텍션 수행
                    results = detector.detect(mediapipe_image)

                    # 검출된 장애물의 종류를 구별해주기
                    self.discriminate_obstacles(results, robot_id)

                    # 검출됭 동적 장애물에 대해 바운딩 박스, fps 그리기
                    self.alba_draw_bbox(decoded_frame, results, info['fps'])

                    # 결과 이미지를 JPEG 형식으로 인코딩
                    retval, encoded_bbox_image = cv2.imencode('.jpg', decoded_frame)

                    if port == 5000 : # MultiThreading 환경에서는 한 번에 여러개의 cv2 창을 띄울 수 없음
                        cv2.imshow("AlbaBot Detection", decoded_frame)
                        cv2.waitKey(1)

                    with self.frame_locks[robot_id]:
                        self.latest_result_frame[robot_id].append(encoded_bbox_image)

            except socket.timeout:
                continue
    
    def tcp_server(self):
        """
        채팅 서버를 위한 TCP 서버를 열어주는 함수입니다.

        Returns :
            None
        """

        TCP_IP = '0.0.0.0'
        TCP_PORT = 8001
        
        tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_socket.bind((TCP_IP, TCP_PORT))
        tcp_socket.listen(5)
        self.get_logger().info(f"🌐 TCP Listening on {TCP_IP}:{TCP_PORT}")
        try :
            while True :
                try:
                    client_socket, client_address = tcp_socket.accept()
                    self.get_logger().info(f"🔌 New TCP connection from {client_address}")
                    hdr = client_socket.recv(4)
                    length = int.from_bytes(hdr, byteorder='big')
                    payload_bytes = b''

                    while len(payload_bytes) < length:
                        chunk = client_socket.recv(length - len(payload_bytes))
                        if not chunk:
                            raise ConnectionError("Incomplete payload")
                        payload_bytes += chunk

                    payload = json.loads(payload_bytes.decode('utf-8'))
                    msg_id = payload.get('msg_id')
                    user_query = payload.get('question')

                    self.get_logger().info(f"📦 Parsed msg_id: {msg_id}")
                    self.get_logger().info(f"📦 Parsed user_query: {user_query}")

                    memory = requests.get(memory_url).json()
                    robot_task = AlbaGPT_function.validate_alba_task_discriminator(user_query, memory, self.llm)
                    robot_id = AlbaGPT_function.extract_robot_id(user_query)

                    self.get_logger().info(f"🧠 Previous memory : {memory}")
                    self.get_logger().info(f"💼 Detected Alba Task : {robot_task}")
                    self.get_logger().info(f"🪪 Detected Alba ID : {robot_id}")

                    if robot_task == "GREETINGS":
                        response = AlbaGPT_function.generate_alba_greetings_response(user_query, memory, self.llm)
                        self.transfer_payloads(msg_id, user_query, robot_id, robot_task, response, None)
                    elif robot_task == "TAKE_PICTURE":
                        for id_port in self.alba_ports.items():
                            id, port = id_port
                            compare_id = "AlbaBot_" + str(robot_id)

                            if id == compare_id :
                                img_path = self.save_obstacle_picture(compare_id)
                                response = AlbaGPT_function.generate_alba_take_picture_response(user_query, memory, self.llm, self.obstacle_info[robot_id]['name'])
                                self.transfer_payloads(msg_id, user_query, robot_id, robot_task, response, img_path)
                    elif robot_task == "MAINTENANCE":
                        response = AlbaGPT_function.generate_alba_maintenance_response(user_query, memory, self.llm)
                        self.transfer_payloads(msg_id, user_query, robot_id, robot_task, response, None)
                    else:
                        response = AlbaGPT_function.generate_alba_none_response(user_query)
                        self.transfer_payloads(msg_id, user_query, robot_id, robot_task, response, None)

                    client_socket.sendall(len(response.encode()).to_bytes(4, 'big') + response.encode())
                    client_socket.close()
                except Exception as e:
                    self.get_logger().warning(f"TCP Error: {e}")
        finally:
            tcp_socket.close()

    def save_obstacle_picture(self, robot_id) :
        """
        지정된 알바봇으로부터 MediaPipe 모델을 통과한 결과 사진을 특정 디렉토리에 저장해주는 함수입니다.

        Returns :
            image_path
        """
        image_dir = './contents/image'
        image_path = os.path.join(image_dir, str(uuid.uuid4().hex) + '_' + time.strftime('%Y-%m-%d %H-%M-%S') + '.jpg')

        if not self.latest_result_frame[robot_id]:
                    self.get_logger().warning(f"🚫 No frame available for {robot_id}")
                    return None
                
        latest_detected_frame = self.latest_result_frame[robot_id].pop()
        
        np_data = np.frombuffer(latest_detected_frame, dtype=np.uint8)
        decoded_detected_image = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

        if decoded_detected_image is None:
            self.get_logger().info("❌ Failed to decode image from decoded_detected_image")
            return None
        else :
            cv2.imwrite(image_path, decoded_detected_image)
            self.get_logger().info(f"📷 Image successfully saved to {image_path}")
            return image_path

    def transfer_payloads(self, msg_id: int, question: str, robot_id: int, robot_task: str, response_text: str, img_path: str):
        """
        채팅 서버로 LLM 응답을 전송해주는 함수입니다.

        Returns :
            None
        """
        CHAT_HOST = "192.168.0.156"    # 채팅 서버 IP
        CHAT_PORT = 8001               # 채팅 서버 포트
        
        base64_img = None

        # 1) 이미지 읽어서 Base64 인코딩
        if img_path is not None and os.path.exists(img_path):
            self.get_logger().info(f"📸 Image found at {img_path}, encoding to Base64...")
            with open(img_path, "rb") as f:
                img_bytes = f.read()
            base64_img = base64.b64encode(img_bytes).decode("utf-8")
            self.get_logger().info(f"✅ Image successfully encoded.")

        # 2) 복합 페이로드 구성
        payload = {
            "msg_type": "Chatbot",
            "msg_id": msg_id,
            "question": question,
            "robot_id": robot_id,
            "robot_task": robot_task,
            "response_text": response_text,
        }

        if base64_img is not None:
            payload["response_image"] = base64_img
        else :
            payload["response_image"] = ""

        raw = json.dumps(payload).encode("utf-8")
        header = len(raw).to_bytes(4, byteorder="big")

        self.get_logger().info(f"📤 Payload contents:\n{json.dumps(payload, indent=4, ensure_ascii=False)}")
        self.get_logger().info(f"🗂 Header prepared: {header.hex()} (length: {len(raw)} bytes)")

        # 3) 페이로드 전송
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((CHAT_HOST, CHAT_PORT))
                self.get_logger().info(f"🌐 Connecting to {CHAT_HOST}:{CHAT_PORT}...")

                sock.sendall(header + raw)
                self.get_logger().info(f"🚀 Payload sent to {CHAT_HOST}:{CHAT_PORT}")

                resp = sock.recv(4096)
                self.get_logger().info(f"⭕️ 서버 응답: {resp.decode()}")

        except Exception as e:
            self.get_logger().error(f"❌ Error during payload transfer: {e}")

    def alba_draw_bbox(self, decoded_frame, results, fps) :
        """
        MediaPipe를 통해 검출된 동적 장애물에 대해서 바운딩 박스 + FPS를 그려주어 결과를 출력해주는 함수입니다.

        Returns :
            None
        """
        for detection in results.detections:
            bbox = detection.bounding_box
            start_point = (int(bbox.origin_x), int(bbox.origin_y))
            end_point = (int(bbox.origin_x + bbox.width), int(bbox.origin_y + bbox.height))

            object = detection.categories[0].category_name
            x = int(detection.bounding_box.origin_x)
            y = int(detection.bounding_box.origin_y)
            width = detection.bounding_box.width
            height = detection.bounding_box.height

            if object in dynamic_object_list :
                cv2.rectangle(decoded_frame, start_point, end_point, (0, 0, 255), 2)
                cv2.putText(decoded_frame, f"{object}", (x + width - 70, y + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            elif object in static_object_list :
                cv2.rectangle(decoded_frame, start_point, end_point, (0, 255, 0), 2)
                cv2.putText(decoded_frame, f"{object}", (x + width - 70, y + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)        
                
        if fps < 20 : 
            cv2.putText(decoded_frame, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        elif fps >= 20 and fps < 60 :
            cv2.putText(decoded_frame, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else :
            cv2.putText(decoded_frame, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    def discriminate_obstacles(self, results, robot_id):
        """
        MediaPipe를 통해 검출된 장애물이 동적 장애물인지 정적 장애물인지 구별해주는 함수입니다.

        Returns :
            None
        """
        if not results.detections :
            self.obstacle_info[robot_id].append("none")
            return
        else :
            for detection in results.detections:
                if detection.categories[0].category_name in dynamic_object_list:
                    self.obstacle_info[robot_id].append("dynamic")
                    return
        # static인 경우
        if detection.categories[0].category_name in static_object_list :
            self.obstacle_info[robot_id].append("static")
            return
        
    def publish_obstacle_information(self):
        """
        최근 몇 프레임에서의 장애물 상태를 바탕으로 다수결 투표 후
        가장 많이 등장한 상태를 퍼블리시합니다.
        """
        for robot_id, info_queue in self.obstacle_info.items():
            if not info_queue:
                continue

            # 다수결 판별
            counter = Counter(info_queue)
            majority_status = counter.most_common(1)[0][0]

            msg = String()
            result_dict = {
                "robot_id": robot_id,
                "type": majority_status,
            }
            msg.data = json.dumps(result_dict, ensure_ascii=False)
            self.get_logger().info(f"💬 {robot_id} || Published (Majority): {result_dict}")
            self.obstacle_information_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        albagpt_node = AlbaGPTNode()
        try:
            rclpy.spin(albagpt_node)
        except KeyboardInterrupt:
            albagpt_node.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            albagpt_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()