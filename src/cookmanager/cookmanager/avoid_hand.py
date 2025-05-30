import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import CookGPTsrv
from collections import deque
import cv2
import numpy as np
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R
import socket
from std_msgs.msg import Bool
import time
import threading
import os
from ament_index_python.packages import get_package_share_directory
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


#불러올 거 불러오기
model_path = os.path.join(get_package_share_directory('cookmanager'), 'hand_landmarker.task')

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = vision.HandLandmarker
HandLandmarkerOptions = vision.HandLandmarkerOptions
VisionRunningMode = vision.RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.IMAGE,
    num_hands=2
)


THRESHOLD_SIZE = 0.4 # 손 크기 TH
LANDMARK_COUNT_THRESHOLD = 12  #손 랜드마크 개수 TH

#노드만들어염
class AvoidHandNode(Node):
    def __init__(self):
        super().__init__('avoid_hand_node')
        
        self.robot_ports = {'robot48': 5002, 'robotb4': 5003}
        self.frames = {name: None for name in self.robot_ports}
        self.frame_locks = {name : threading.Lock() for name in self.robot_ports}
        self.hand_publishers = {}

        self.landmarker = HandLandmarker.create_from_options(options)

        for name in self.robot_ports:
            topic_name = f'/{name}/hand_detected'
            self.hand_publishers[name] = self.create_publisher(Bool, topic_name , 10)

        for name, port in self.robot_ports.items():
            threading.Thread(target=self.udp_loop, args=(name, port), daemon=True).start()

        #토픽발행하는 주기
        self.timer = self.create_timer(1.0/30.0 , self.detect_and_publish)
        self.get_logger().info("CookGPT - Avoid Hand 노드 토픽 서버 대기하는중~~~~~~")


    def udp_loop(self, robot_name, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', port))
        sock.settimeout(1.0)
        while True:
            try:
                packet, _ = sock.recvfrom(65536)
                jpeg_data = packet[8:]
                frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    with self.frame_locks[robot_name]:
                        self.frames[robot_name] = frame.copy()
            except socket.timeout:
                continue

    def detect_and_publish(self):
        for name in self.robot_ports:
            with self.frame_locks[name]:
                frame = self.frames[name]
            if frame is None:
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data =rgb)
            result = self.landmarker.detect(mp_image)

            detected = False
            
            #손에도 TH 적용해염 (완전작은 손도 잘보여서 걸엇어요 가끔 멀리잇는 사람얼굴도 손으로 인지함;;)
            if result.hand_landmarks:
                for landmarks in result.hand_landmarks:
                    xs = [lm.x for lm in landmarks]
                    ys = [lm.y for lm in landmarks]
                    width = max(xs) - min(xs)
                    height = max(ys) - min(ys)
                    if max(width, height) >= THRESHOLD_SIZE and len(landmarks) >= LANDMARK_COUNT_THRESHOLD:
                        detected = True
                        break

            msg = Bool()
            msg.data = detected
            self.hand_publishers[name].publish(msg)
            self.get_logger().info(f"{name} 손 감지: {detected}")

            
def main(args=None):
    rclpy.init(args=args)
    node = AvoidHandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
