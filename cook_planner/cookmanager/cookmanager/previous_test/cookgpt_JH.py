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
        pkg_path = get_package_share_directory('cookmanager')
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
                # self.get_logger().info(f"✅ {robot_name}: 수신됨")  
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
            self.get_logger().warn(f"❌ 알 수 없는 로봇 ID 요청: {robot_id}")
            return response

        if cmd in [0, 1, 2, 3]:
            R_flip = R.from_euler('x', [180], degrees=True).as_matrix()
            keypoints_list = []
            collected = 0
            max_frame_attempts = 20

            try:
                for _ in range(max_frame_attempts):
                    with self.frame_locks[robot_id]:
                        frame = self.latest_frames.get(robot_id, None)

                    if frame is None:
                        time.sleep(0.05)
                        continue

                    results = model(frame, verbose=False)[0]

                    if results.keypoints is None or results.keypoints.xy is None:
                        continue

                    # ✅ 여러 감지된 물체 중에서 y값이 가장 작은 (가까운) 물체 선택
                    best_kpts = None
                    min_y_avg = float('inf')

                    for i in range(len(results.keypoints.xy)):
                        if i >= len(results.boxes.cls):  # 방어코드
                            continue

                        kp_class = int(results.boxes.cls[i].item())
                        if kp_class != cmd:
                            continue

                        kpts = results.keypoints.xy[i].cpu().numpy().astype(np.float32)
                        if kpts.shape[0] < 4:
                            continue

                        y_avg = np.mean(kpts[:4, 1])
                        if y_avg < min_y_avg:
                            best_kpts = kpts[:4]
                            min_y_avg = y_avg

                    if best_kpts is not None:
                        keypoints_list.append(best_kpts)
                        collected += 1

                    if collected >= 5:
                        break

                if collected < 4:
                    self.get_logger().warn(f"{robot_id} - 클래스 {cmd} keypoint 세트 부족 ({collected})")
                    return response

                keypoints_np = np.mean(np.stack(keypoints_list), axis=0)  # shape: (4, 2)
                object_points_np = np.array(objp[:4], dtype=np.float32)
                image_points_np = keypoints_np.astype(np.float32)

                ret, rvec, tvec = cv2.solvePnP(object_points_np, image_points_np, camera_matrix, dist_coeffs)

                if not ret:
                    self.get_logger().warn(f"{robot_id} - solvePnP 실패")
                    return response

                R_obj_cam, _ = cv2.Rodrigues(rvec)
                R_obj_cam_flipped = R_flip @ R_obj_cam
                rpy = R.from_matrix(np.squeeze(R_obj_cam_flipped)).as_euler('xyz', degrees=True)

                # self.get_logger().info(f"[DEBUG] rvec = {rvec.flatten()}")
                # self.get_logger().info(f"[DEBUG] R_obj_cam =\n{R_obj_cam}")
                # self.get_logger().info(f"[DEBUG] R_obj_cam_flipped =\n{R_obj_cam_flipped}")

                if rpy.shape != (3,) or np.isnan(rpy).any():
                    self.get_logger().warn(f"{robot_id} - RPY 변환 실패 또는 NaN 발생")
                    return response

                response.x, response.y, response.z = tvec.flatten()
                response.rx, response.ry, response.rz = rpy

                response.dish = False
                response.sauce = False
                response.stain = False

                self.get_logger().info(f"✅ {response} response")

            except Exception as e:
                self.get_logger().warn(f"{robot_id}❌ YOLO/solvePnP 실패: {e}")
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

if __name__ == '__main__':
    main()