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
from concurrent.futures import ThreadPoolExecutor

# ✅ 파일 경로 확보
pkg_path = get_package_share_directory('cookM')

objp = np.array([
    [-15, 15, 0],
    [15, 15, 0],
    [15, -15, 0],
    [-15, -15, 0]
], dtype=np.float32)

# 모델 로드
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
        self.robot_ports = {'robot48': 5002, 'robotb4': 5001}
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

        self.srv = self.create_service(CookGPTsrv, 'CookGPTsrv', self.handle_request)

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
                    # self.latest_frames[robot_name] = imgd  # :white_check_mark: 이미지만 저장
                    self.latest_frames[robot_name].append(imgd)
            except socket.timeout:
                continue


    def handle_request(self, request, response):
        robot_id = request.robot_id
        camera_matrix, dist_coeffs = self.camera_params[robot_id]
        cmd = request.command

        if robot_id not in self.robot_ports:
            self.get_logger().warn(f":x: 알 수 없는 로봇 ID 요청: {robot_id}")
            return response

        #5프레임 쌓일 때까지 기둘
        frames = []
        while True:
            with self.frame_locks[robot_id]:
                frames = list(self.latest_frames.get(robot_id, []))
            if len(frames) >= 5:
                break
            time.sleep(0.01)

        if cmd in [0, 1, 2, 3]:
            tvecs = []
            rvecs = []

            try:
                for frame in frames:
                    results = model(frame, verbose=False)[0]
                    filtered = []

                    # 원하는 클래스만 추출
                    if results.keypoints.cls is None:
                        continue

                    for i, kp in enumerate(results.keypoints.xy):
                        kp_class = int(results.keypoints.cls[i].item())
                        if kp_class != cmd:
                            continue

                        kpt = kp.cpu().numpy()[:4].astype(np.float32)
                        ret, rvec, tvec = cv2.solvePnP(objp, kpt, camera_matrix, dist_coeffs)
                        if ret:
                            filtered.append((tvec, rvec))

                    if not filtered:
                        continue

                    # Y축 기준 가장 가까운 객체 선택
                    closest = min(filtered, key=lambda x: x[0][1])
                    tvecs.append(closest[0])
                    rvecs.append(closest[1])

                if len(tvecs) == 0:
                    self.get_logger().warn(f"{robot_id} - pose 없음 (cmd {cmd})")
                    return response

                # tvec 평균
                tvecs_np = np.array(tvecs).reshape(-1, 3)
                avg_tvec = np.mean(tvecs_np, axis=0)

                # rvec → quaternion → 평균 → RPY
                quats = []
                for rvec in rvecs:
                    R_mat, _ = cv2.Rodrigues(rvec)
                    R_flipped = R.from_euler('x', [180], degrees=True).as_matrix() @ R_mat
                    quat = R.from_matrix(R_flipped).as_quat()  # [x, y, z, w]
                    quats.append(quat)

                quats_np = np.array(quats)
                quat_mean = np.mean(quats_np, axis=0)
                quat_mean /= np.linalg.norm(quat_mean)
                rpy = R.from_quat(quat_mean).as_euler('xyz', degrees=True)

                response.x, response.y, response.z = avg_tvec
                response.rx, response.ry, response.rz = rpy
                self.get_logger().info(f"✅ {robot_id} - 평균 pose 추정 완료")

            except Exception as e:
                self.get_logger().warn(f"YOLO/solvePnP 평균 오류: {e}")
                return response

        elif cmd in [4, 5, 6]:
            label_map = {4: "dish", 5: "sauce", 6: "stain"}
            label = label_map[cmd]
            try:
                results = model(frames[-1], verbose=False)[0]
                found = any([
                    int(r[5]) == model.names.index(label)
                    for r in results.boxes.data.cpu().numpy()
                ])
                setattr(response, label, found)
                self.get_logger().info(f":mag_right: {robot_id} - {label} 존재 여부: {found}")
            except Exception as e:
                self.get_logger().warn(f":x: {label} 탐지 실패: {e}")

        return response
    
    def prevent_collision(self):


        return
    
    def imshow_loop(self):
        print("✅ imshow_loop 실행됨")
        while True:
            for name in self.robot_ports.keys():
                with self.frame_locks[name]:
                    frame = self.latest_frames.get(name, None)
                if frame is not None:
                    print(f"🖼️ {name} 프레임 수신됨")
                    cv2.imshow(f'{name} view', frame)
            if cv2.waitKey(1) == ord('q'):
                break
        cv2.destroyAllWindows()

# def main(args=None):
#     rclpy.init(args=args)
#     node = CookGPTServiceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# # imshow 수신 확인용 
def main(args=None):
    
    print("👋 main() 진입 완료", flush=True)
    rclpy.init(args=args)
    node = CookGPTServiceNode()

    # ROS spin을 백그라운드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # imshow GUI 루프는 메인에서
    node.imshow_loop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

