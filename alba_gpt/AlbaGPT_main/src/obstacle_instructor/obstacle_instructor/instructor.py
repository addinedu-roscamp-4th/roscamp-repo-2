import rclpy
import mediapipe as mp
import threading
import socket
import cv2
import numpy as np
import time
import json

from collections import deque
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String

dynamic_object_list = ["person", "bird", "cat", "dog"]

class AlbaGPTNode(Node):
    def __init__(self):
        super().__init__('albagpt_node')
        self.alba_ports = {'AlbaBot_1': 5000, 'AlbaBot_2': 5001, 'AlbaBot_3': 5002}
        self.declare_parameter('qos_depth', 10)
        self.latest_result_frame = {name: deque(maxlen=5) for name in self.alba_ports}
        self.frame_locks = {name: threading.Lock() for name in self.alba_ports} # 해당 쓰레드만 
        self.obstacle_type = {
            name: "" for name in self.alba_ports
        }
        self.fps_info = {
            name: {'cnt': 0, 'start_time': time.time(), 'fps': 0}
            for name in self.alba_ports
        }       

        # 스레드 시작
        for id_port in self.alba_ports.items():
            robot_id, port = id_port
            udp_server = threading.Thread(target=self.udp_server, args=(robot_id, port), daemon=True)
            udp_server.start()
            self.get_logger().info(f"📡 {robot_id}:{port}의 카메라 수신 스레드 시작됨")

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

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        self.get_logger().info(f"🌐 {robot_id} || UDP Listening on {UDP_IP}:{UDP_PORT}")

        while True:
            try:
                packet, addr = sock.recvfrom(BUFFER_SIZE)
                np_data = np.frombuffer(packet, dtype=np.uint8)
                decoded_frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

                if decoded_frame is None:
                    continue

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

                # 결과 출력
                cv2.imshow(f"{robot_id} || {UDP_IP}:{UDP_PORT} Streaming...", decoded_frame)
                cv2.waitKey(1)

                # 결과 이미지를 JPEG 형식으로 인코딩
                retval, encoded_bbox_image = cv2.imencode('.jpg', decoded_frame)

                with self.frame_locks[robot_id]:
                    self.latest_result_frame[robot_id].append(encoded_bbox_image)

            except socket.timeout:
                continue

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
            self.obstacle_type[robot_id] = "none"
            return
        else :
            for detection in results.detections:
                if detection.categories[0].category_name in dynamic_object_list:
                    self.obstacle_type[robot_id] = "dynamic"
                    return
        self.obstacle_type[robot_id] = "static"
        
    def publish_obstacle_information(self):
        """
        MediaPipe를 통해 검출된 장애물의 종류를 토픽으로 전송해주는 함수입니다.

        Returns :
            None
        """
        for robot_id, obstacle_type in self.obstacle_type.items():
            msg = String()
            result_dict = {
                "robot_id": robot_id,
                "obstacle_type": obstacle_type,
            }
            msg.data = json.dumps(result_dict)

            self.get_logger().info(f"💬 {result_dict["robot_id"]} || Published message : {result_dict["obstacle_type"]}")
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