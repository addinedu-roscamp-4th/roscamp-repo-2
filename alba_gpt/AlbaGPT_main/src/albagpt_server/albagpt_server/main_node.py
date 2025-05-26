import mediapipe as mp
import threading
import socket
import cv2
import numpy as np
import time
import json
import base64
import rclpy
import os
import sounddevice as sd
import whisper

from scipy.io.wavfile import write
from . import AlbaGPT_function
from itertools import islice
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
from pinky_interfaces.srv import Pinkytask

ChatOpenAI.model_rebuild()

class AlbaGPTNode(Node):
    def __init__(self):
        super().__init__('albagpt_node')
        self.alba_ports = {'AlbaBot_1': 5000, 'AlbaBot_2': 5001, 'AlbaBot_3': 5002}
        self.declare_parameter('qos_depth', 8)
        self.latest_result_frame = {name: deque(maxlen=5) for name in self.alba_ports}
        self.frame_locks = {name: threading.Lock() for name in self.alba_ports} # 해당 쓰레드만 
        self.obstacle_type = {
            name: deque(maxlen=40) for name in self.alba_ports
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
        
        pinky_interaction_thread = threading.Thread(target=self.pinky_interaction, daemon=True)
        pinky_interaction_thread.start()
        self.get_logger().info("🗪 Pinky Interaction thread started.")

        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth)

        self.obstacle_information_publisher = self.create_publisher(
            String,
            'discriminated_obstacle',
            QOS_RKL10V)

        self.publish_timer = self.create_timer(1.0, self.publish_obstacle_information)

        self.client = self.create_client(Pinkytask, 'set_task')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for service server...')

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

                    if port == 5002 : # MultiThreading 환경에서는 한 번에 여러개의 cv2 창을 띄울 수 없음
                        cv2.imshow("AlbaBot Detection", decoded_frame)
                        cv2.waitKey(1)

                    with self.frame_locks[robot_id]:
                        self.latest_result_frame[robot_id].append(encoded_bbox_image)

            except socket.timeout:
                continue
    
    def pinky_interaction(self):
        """
        핑키와의 인터렉션을 위해 Whisper 모델을 활용하여 유저의 입력을 해석하여 명령을 전달해주는 함수입니다.

        Returns :
            None
        """
        # MIC & whisper Setup
        samplerate = 16000
        duration = 5
        tts_path = "./contents/mic/tts.wav"
        model = whisper.load_model("tiny")

        while True:
            try:
                self.get_logger().info("🎙️ 녹음 시작...")
                recording = sd.rec(int(samplerate * duration), samplerate=samplerate, channels=1, dtype='int16')
                sd.wait()
                write(tts_path, samplerate, recording)
                self.get_logger().info("✅ 녹음 완료")

                result = model.transcribe(f"{tts_path}", language="ko")
                
                user_query = result.get("text", "").strip()

                if user_query:
                    self.get_logger().info(f"📝 변환된 텍스트: {user_query}")
                    robot_task = AlbaGPT_function.validate_alba_task_discriminator(user_query, self.llm)
                    robot_id = AlbaGPT_function.extract_robot_id(user_query)

                    self.get_logger().info(f"💼 Detected Alba Task : {robot_task}")
                    self.get_logger().info(f"🪪 Detected Alba ID : {robot_id}")

                    request = Pinkytask.Request()
                    request.robot_id = int(robot_id)
                    request.robot_task = str(robot_task)

                    self.future = self.client.call_async(request)  # 비동기 호출
                    self.get_logger().info("🚀 서비스 요청 전송 완료")
                else:
                    self.get_logger().info("⚠️ Whisper가 아무 텍스트도 추출하지 못했습니다.")
            except Exception as e:
                self.get_logger().error(f"❌ 예외 발생: {e}")
                continue

    def alba_draw_bbox(self, decoded_frame, results, fps) :
        """
        MediaPipe를 통해 검출된 동적 / 정적 장애물에 대해서 바운딩 박스 + FPS를 그려주어 결과를 출력해주는 함수입니다.

        Returns :
            None
        """
        if results.detections : 
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
                    
        if fps < 15 : 
            cv2.putText(decoded_frame, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        elif fps < 30 :
            cv2.putText(decoded_frame, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else :
            cv2.putText(decoded_frame, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    def discriminate_obstacles(self, results, robot_id):
        """
        MediaPipe를 통해 검출된 장애물이 동적 장애물인지 정적 장애물인지 구별해주는 함수입니다.
        만약 감지된 장애물이 없으면 "none"으로 구별해줍니다.

        Returns :
            None
        """
        if not results.detections:
            self.obstacle_type[robot_id].appendleft("none")
            return

        for detection in results.detections:
            obj_name = detection.categories[0].category_name
            if obj_name in dynamic_object_list:
                self.obstacle_type[robot_id].appendleft("dynamic")
                return
            elif obj_name in static_object_list:
                self.obstacle_type[robot_id].appendleft("static")
                return
            
    def publish_obstacle_information(self):
        """
        최근 몇 프레임에서의 장애물 상태를 바탕으로 다수결 투표 후
        가장 많이 등장한 타입(type)을 퍼블리시합니다.

        Returns :
            None
        """
        for robot_id, type_queue in self.obstacle_type.items():
            types = []

            if len(type_queue) >= 30 :
                types = list(islice(type_queue, 30))
            else :
                continue
            
             # 장애물의 타입을 비율 기준으로 처리
            type_counter = Counter(types)
            total = len(types)
            type_percent = {
                key: (value / total) * 100 for key, value in type_counter.items()
            }

            majority_type = "none"  # 기본값

            dynamic_percent = type_percent.get("dynamic", 0)
            static_percent = type_percent.get("static", 0)
            none_percent = type_percent.get("none", 0)

            if dynamic_percent >= 30: # 큐에 들어있는 값들 중 30% 이상이 dynamic이면 해당 인자로 분류
                majority_type = "dynamic"
            if static_percent >= 50: # 큐에 들어있는 값들 중 50% 이상이 static이면 static으로 분류
                majority_type = "static"
            if none_percent == 100 : # 큐에 들어있는 값이 전부 none이면 none으로 분류
                majority_type = "none"

            result_dict = {
                "robot_id": robot_id,
                "type": majority_type,
            }

            msg = String()
            msg.data = json.dumps(result_dict, ensure_ascii=False)
            self.get_logger().info(f"Queue Size : {len(type_queue)}")
            self.get_logger().info(f"Dynamic : {dynamic_percent} || Static : {static_percent} || None : {none_percent}")
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