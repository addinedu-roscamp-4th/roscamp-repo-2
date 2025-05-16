import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import CookGPTsrv
from collections import deque
import cv2
import numpy as np
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R
import socket
import struct
import time
import threading
import os
from ament_index_python.packages import get_package_share_directory
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2


#불러올 거 불러오기
pkg_path = get_package_share_directory('cookM')
model_path = 'hand_landmarker.task'  # 실제 경로로 바꿔주세요
recording = False
writer = None

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.IMAGE,
    num_hands=2
)


THRESHOLD_SIZE = 0.4 # 손 크기 임계값
LANDMARK_COUNT_THRESHOLD = 12

#노드만들어염
class AvoidHandNode(Node):
    def __init__(self):
        super().__init__('avoid_hand_node')
        self.frame_locks = {name: threading.Lock() for name in self.robot_ports}
        self.robot_ports = {'robot48': 5000, 'robotb4': 5001}
        pkg_path = get_package_share_directory('cookM')


        for name, port in self.robot_ports.items():
            t = threading.Thread(target=self.udp_loop, args=(name, port), daemon=True)
            t.start()
            self.get_logger().info(f"{name} 카메라 수신 스레드 시작됨 (port {port})")
        self.srv = self.create_service(CookGPTsrv, 'CookGPTsrv', self.handle_request)
        self.get_logger().info("CookGPT - Avoid Hand 노드 서비스 서버 대기 중...")


    def udp_loop(self, robot_name, port):
        camera_matrix, dist_coeffs = self.camera_params[robot_name]
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', port))
        sock.settimeout(1.0)
        while True:
            try:
                packet, _ = sock.recvfrom(65536)
                if len(packet) <= 8:
                    continue
                jpeg_data = packet[8:]
                npdata = np.frombuffer(jpeg_data, dtype=np.uint8)
                self.frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)
                if self.frame is None:
                    continue
            except socket.timeout:
                continue

    def handle_request(self, request, response):
        robot_id = request.robot_id

        with HandLandmarker.create_from_options(options) as landmarker:
            rgb_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            result = landmarker.detect(mp_image)

            detection_flag = 0
            response = detection_flag

        if result.hand_landmarks:
            for landmarks in result.hand_landmarks:
                # 1. 손 크기 계산
                xs = [lm.x for lm in landmarks]
                ys = [lm.y for lm in landmarks]
                width = max(xs) - min(xs)
                height = max(ys) - min(ys)
                hand_size = max(width, height)


                if hand_size < THRESHOLD_SIZE:
                    continue

                if len(landmarks) >= LANDMARK_COUNT_THRESHOLD:
                    detection_flag = 1
                    response = detection_flag

        return response


