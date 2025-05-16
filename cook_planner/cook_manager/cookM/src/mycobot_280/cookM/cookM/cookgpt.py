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
# :흰색_확인_표시: 파일 경로 확보
pkg_path = get_package_share_directory('cookM')
objp = np.array([
    [-15, 15, 0],
    [15, 15, 0],
    [15, -15, 0],
    [-15, -15, 0]
], dtype=np.float32)
# :흰색_확인_표시: 모델 로드도 안정적으로 처리하고 싶다면 여기도 절대 경로 사용 가능
model_path = os.path.join(pkg_path, 'best.pt')
model = YOLO(model_path)
class CookGPTServiceNode(Node):
    def __init__(self):
        super().__init__('cookgpt_service_node')
        # self.robot_ports = {'robot48': 5000, 'robotb4': 5001}
        self.robot_ports = {'robot48': 5002, 'robotb4': 5001}
        # self.latest_frames = {}
        self.latest_frames = {name: deque(maxlen=5) for name in self.robot_ports}
        self.frame_locks = {name: threading.Lock() for name in self.robot_ports}
        self.latest_pose = {}
        self.get_pose = {name: False for name in self.robot_ports}
        # :흰색_확인_표시: 로봇별 calibration 로딩
        pkg_path = get_package_share_directory('cookM')
        self.calibrations = {
            'robot48': np.load(os.path.join(pkg_path, 'calibration_aa48.npz')),
            'robotb4': np.load(os.path.join(pkg_path, 'calibration_refined4.npz')),
        }
        self.camera_params = {
            name: (data["camera_matrix"], data["dist_coeffs"])
            for name, data in self.calibrations.items()
        }
        # 스레드 시작
        for name, port in self.robot_ports.items():
            t = threading.Thread(target=self.udp_loop, args=(name, port), daemon=True)
            t.start()
            self.get_logger().info(f":위성_안테나: {name} 카메라 수신 스레드 시작됨 (port {port})")
        self.srv = self.create_service(CookGPTsrv, 'CookGPTsrv', self.handle_request)
        self.get_logger().info(":흰색_확인_표시: CookGPT 서비스 서버 대기 중...")
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
                frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                imgd = cv2.undistort(frame, camera_matrix, dist_coeffs)
                with self.frame_locks[robot_name]:
                    self.latest_frames[robot_name] = imgd  # :흰색_확인_표시: 이미지만 저장
            except socket.timeout:
                continue
    def handle_request(self, request, response):
        robot_id = request.robot_id
        camera_matrix, dist_coeffs = self.camera_params[robot_id]
        cmd = request.command
        if robot_id not in self.robot_ports:
            self.get_logger().warn(f":x: 알 수 없는 로봇 ID 요청: {robot_id}")
            return response
        # :열쇠와_잠긴_자물쇠: 프레임만 안전하게 꺼냄
        while True:
            with self.frame_locks[robot_id]:
                frames = self.latest_frames.get(robot_id, None)
            if len(frames) >= 5:
                break
            # if frame is None:
            #     self.get_logger().warn(f":경고: {robot_id} 프레임 없음")
            #     return response
        # :압정: cmd 0~3: pose 추정 요청
        if cmd in [0, 1, 2, 3]:
            pose = None
            all_keypoints = []
            R_flip = R.from_euler('x', [180], degrees=True).as_matrix()
            try:
                for frame in frames:
                    results = model(frame, verbose=False)[0]
                    if results.keypoints.cls is None:
                        continue
                    for i, kp in enumerate(results.keypoints.xy):
                        kp_class = int(results.keypoints.cls[i].item())
                        if kp_class != cmd:
                            continue
                        kpt = kp.cpu().numpy()[:4].astype(np.float32)
                        all_keypoints.append(kpt)
                if len(all_keypoints) == 0:
                    self.get_logger().warn(f"{robot_id} - 클래스 {cmd} keypoint 감지 실패")
                    return response
                keypoints_avg = np.mean(np.array(all_keypoints), axis=0)
                ret, rvec, tvec = cv2.solvePnP(objp, keypoints_avg, camera_matrix, dist_coeffs)
                if not ret:
                    self.get_logger().warn(f"{robot_id} - solvePnP 실패")
                    return response
                # RPY 계산 (quaternion averaging 아님, 1회 solvePnP라서 필요 없음요)
                R_obj_cam, _ = cv2.Rodrigues(rvec)
                R_obj_cam_flipped = R_flip @ R_obj_cam
                rpy = R.from_matrix(R_obj_cam_flipped).as_euler('xyz', degrees=True)
                if rpy.shape != (3,) or np.isnan(rpy).any():
                    self.get_logger().warn(f"{robot_id} - RPY 변환 실패 또는 NaN 발생")
                    return response
                response.x, response.y, response.z = tvec.flatten()
                response.rx, response.ry, response.rz = rpy
                self.get_logger().info(f"데이터 : {response}")
            except Exception as e:
                self.get_logger().warn(f":x: YOLO/solvePnP 실패: {e}")
        # :압정: cmd 4~6: 객체 탐지 결과 응답
        elif cmd in [4, 5, 6]:
            label_map = {4: "dish", 5: "sauce", 6: "stain"}
            label = label_map[cmd]
            try:
                results = model(frame, verbose=False)[0]
                found = any([
                    int(r[5]) == model.names.index(label)
                    for r in results.boxes.data.cpu().numpy()
                ])
                setattr(response, label, found)
                self.get_logger().info(f":렌즈가_오른쪽_위에_있는_확대경: {robot_id} - {label} 존재 여부: {found}")
            except Exception as e:
                self.get_logger().warn(f":x: {label} 탐지 실패: {e}")
        return response
def main(args=None):
    rclpy.init(args=args)
    node = CookGPTServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()





