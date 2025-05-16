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
from rclpy.executors import MultiThreadedExecutor

# ✅ 파일 경로 확보
pkg_path = get_package_share_directory('cookM')

objp = np.array([
    [-15, 15, 0],
    [15, 15, 0],
    [15, -15, 0],
    [-15, -15, 0]
], dtype=np.float32)

# ✅ 모델 로드도 안정적으로 처리하고 싶다면 여기도 절대 경로 사용 가능
model_path = os.path.join(pkg_path, 'best.pt')
model = YOLO(model_path)
model.to('cuda')

# warm-up (더미로 미리 1번 추론)
dummy = np.zeros((640, 640, 3), dtype=np.uint8)
_ = model(dummy, verbose=False)[0]

class CookGPTServiceNode(Node):
    def __init__(self):
        super().__init__('cookgpt_service_node')
        # self.robot_ports = {'robot48': 5000, 'robotb4': 5001}
        self.robot_ports = {'robot48': 5000, 'robotb4': 5001}
        # self.latest_frames = {}
        self.latest_frames = {name: deque(maxlen=5) for name in self.robot_ports}
        self.frame_locks = {name: threading.Lock() for name in self.robot_ports}

        # ✅ 로봇별 calibration 로딩
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
            self.get_logger().info(f"📡 {name} 카메라 수신 스레드 시작됨 (port {port})")

        self.srv = self.create_service(CookGPTsrv, 'CookGPTsrv', self.handle_request)
        self.get_logger().info("✅ CookGPT 서비스 서버 대기 중...")        

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
                    self.latest_frames[robot_name] = imgd  # ✅ 이미지만 저장

            except socket.timeout:
                continue


    def handle_request(self, request, response):
        robot_id = request.robot_id
        camera_matrix, dist_coeffs = self.camera_params[robot_id]
        cmd = request.command

        if robot_id not in self.robot_ports:
            self.get_logger().warn(f"알 수 없는 로봇 ID 요청: {robot_id}")
            return response

        if cmd in [0, 1, 2, 3]:
            R_flip = R.from_euler('x', [180], degrees=True).as_matrix()
            tvecs = []
            rvecs = []
            attempts = 0
            max_attempts = 30

            try:
                while len(tvecs) < 5 and attempts < max_attempts:
                    with self.frame_locks[robot_id]:
                        frame = self.latest_frames.get(robot_id, None)

                    if frame is None:
                        time.sleep(0.05)
                        attempts += 1
                        continue

                    results = model(frame, verbose=False)[0]

                    if results.keypoints is None or results.keypoints.xy is None:
                        attempts += 1 
                        continue

                    # 여러 감지된 물체 중에서 y기준으로 가까운 물체 선택

                    best_pose = None
                    min_y = float('inf')

                    for i,kp in enumerate(results.keypoints.xy): # 모든 객체들의 키포인트들  
                        if results.keypoints.cls is None or i >= len(results.keypoints.cls):  # 방어코드
                            continue

                        kp_class = int(results.keypoints.cls[i].item())
                        if kp_class != cmd:
                            continue

                        kpts = kp.cpu().numpy().astype(np.float32)
                        if kpts.shape[0] < 4:
                            continue

                        image_points = kpts[:4]   #얘는 없어도되긴함~~~~~
                        ret, rvec, tvec = cv2.solvePnP(objp, image_points, camera_matrix, dist_coeffs)
                        if not ret:
                            continue

                        if tvec[1] < min_y :
                            best_pose = (tvec,rvec)
                            min_y = tvec[1]

                    #여기 best_pose는 6D임 6D
                    if best_pose:
                        tvecs.append(best_pose[0])
                        rvecs.append(best_pose[1])

                    attempts += 1

                if len(tvecs) < 5:
                    self.get_logger().warn(f"{robot_id} - 5개 포즈 수집 실패 (성공 {len(tvecs)}개, 시도 {attempts}회)")
                    return response
                
                tvec_avg = np.mean(np.array(tvecs).reshape(-1, 3), axis=0)

                quats = []
                for rvec in rvecs:
                    R_mat, _ = cv2.Rodrigues(rvec)
                    R_flipped = R_flip @ R_mat
                    quat = R.from_matrix(R_flipped).as_quat()
                    quats.append(quat)

                quat_avg = np.mean(np.array(quats), axis = 0)
                quat_avg /= np.linalg.norm(quat_avg)
                rpy = R.from_quat(quat_avg).as_euler('xyz', degrees=True)

                response.x, response.y, response.z = tvec_avg
                response.rx, response.ry, response.rz = rpy

                response.dish = False
                response.sauce = False
                response.stain = False

                self.get_logger().info(f"{robot_id} 평균 solvePnP 완")

            except Exception as e:
                self.get_logger().warn(f"{robot_id} solvePnP 과정 실패: {e}")
                return response

        elif cmd in [4, 5, 6]:
            label_map = {4: "dish", 5: "sauce", 6: "stain"}
            label = label_map[cmd]

            try:
                with self.frame_locks[robot_id]:
                    frame = self.latest_frames.get(robot_id, None)

                if frame is None:
                    self.get_logger().warn(f"⚠️ {robot_id} 프레임 없음")
                    return response

                results = model(frame, verbose=False)[0]
                found = any([
                    int(r[5]) == model.names.index(label)
                    for r in results.boxes.data.cpu().numpy()
                ])
                setattr(response, label, found)
                self.get_logger().info(f"🔎 {robot_id} - {label} 존재 여부: {found}")
                response.dish = True
                response.sauce = True
                response.stain = True
            except Exception as e:
                response.dish = False
                response.sauce = False
                response.stain = False
                self.get_logger().warn(f"❌ {label} 탐지 실패: {e}")

        self.get_logger().info(f"✅ {robot_id} solvePnP 결과 전송됨")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CookGPTServiceNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

# imshow 수신 확인용 
# def main(args=None):
    
#     print("👋 main() 진입 완료", flush=True)
#     rclpy.init(args=args)
#     node = CookGPTServiceNode()

#     # ROS spin을 백그라운드에서 실행
#     spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     spin_thread.start()

#     # imshow GUI 루프는 메인에서
#     node.imshow_loop()

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()