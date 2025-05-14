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
        
        frames = []

        #5장받을때까지 대기
        while True:
            with self.frame_locks[robot_id]:
                frames = list(self.latest_frames.get(robot_id, []))

            if len(frames) >= 5:
                break
            time.sleep(0.01)


        if cmd in [0, 1, 2, 3]:
            all_keypoints = []

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
                R_obj_cam_flipped = R.from_euler('x', [180], degrees=True).as_matrix() @ R_obj_cam
                rpy = R.from_matrix(R_obj_cam_flipped).as_euler('xyz', degrees=True)

                if rpy.shape != (3,) or np.isnan(rpy).any():
                    self.get_logger().warn(f"{robot_id} - RPY 변환 실패 또는 NaN 발생")
                    return response

                response.x, response.y, response.z = tvec.flatten()
                response.rx, response.ry, response.rz = rpy
                self.get_logger().info(f"{robot_id} - cmd {cmd} pose 추정 완료")

            except Exception as e:
                self.get_logger().warn(f"YOLO/solvePnP 오류: {e}")
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

#추가해야 할 것 : class 받아서, 일치하는 것들에만 solvePnP 적용하고 , 가장 가까운 6D만 전달하기
# class를 우째받아염
# 주는 건 어떻게 따로줘염

#rx ry rz 를 음수 양수 개수에 따라 3개이상인 걸로 통일
