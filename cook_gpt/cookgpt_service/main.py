import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import CookGPTsrv

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

# ✅ 파일 경로 확보
pkg_path = get_package_share_directory('cookM')
calib_path = os.path.join(pkg_path, 'calibration_refined4.npz')

# ✅ 캘리브레이션 데이터 로드
calib = np.load(calib_path)
camera_matrix = calib["camera_matrix"]
dist_coeffs = calib["dist_coeffs"]

objp = np.array([
    [-15, 15, 0],
    [15, 15, 0],
    [15, -15, 0],
    [-15, -15, 0]
], dtype=np.float32)

# ✅ 모델 로드도 안정적으로 처리하고 싶다면 여기도 절대 경로 사용 가능
model_path = os.path.join(pkg_path, 'best.pt')
model = YOLO(model_path)

class CookGPTServiceNode(Node):
    def __init__(self):
        super().__init__('cookgpt_service_node')
        self.robot_ports = {'robot48': 5000, 'robotb4': 5001}
        self.latest_frames = {}
        self.latest_results = {}
        self.latest_pose = {}
        self.get_pose = {name: False for name in self.robot_ports}
        self.frame_locks = {name: threading.Lock() for name in self.robot_ports}
        
        for name, port in self.robot_ports.items():
            t = threading.Thread(target=self.udp_loop, args=(name, port), daemon=True)
            t.start()
            self.get_logger().info(f"📡 {name} 카메라 수신 스레드 시작됨 (port {port})")


        self.srv = self.create_service(CookGPTsrv, 'CookGPTsrv', self.handle_request)
        self.get_logger().info("✅ CookGPT 서비스 서버 대기 중...")

    def udp_loop(self, robot_name, port):
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
                    self.latest_frames[robot_name] = imgd  # :white_check_mark: 이미지만 저장
            except socket.timeout:
                continue
    def handle_request(self, request, response):
        robot_id = request.robot_id
        cmd = request.command
        if robot_id not in self.robot_ports:
            self.get_logger().warn(f":x: 알 수 없는 로봇 ID 요청: {robot_id}")
            return response
        # :closed_lock_with_key: 프레임만 안전하게 꺼냄
        with self.frame_locks[robot_id]:
            frame = self.latest_frames.get(robot_id, None)
        if frame is None:
            self.get_logger().warn(f":warning: {robot_id} 프레임 없음")
            return response
        # :pushpin: cmd 0~3: pose 추정 요청
        if cmd in [0, 1, 2, 3]:
            pose = None
            R_flip = R.from_euler('x', [180], degrees=True).as_matrix()
            try:
                results = model(frame, verbose=False)[0]  # :white_check_mark: YOLO 추론
                if results.keypoints is not None and len(results.keypoints) > 0:
                    imgp = results.keypoints[0].xy.cpu().numpy()[:4].astype(np.float32)
                    ret, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs)
                    if ret:
                        R_obj_cam, _ = cv2.Rodrigues(rvec)
                        R_obj_cam_flipped = R_flip @ R_obj_cam
                        rpy = R.from_matrix(R_obj_cam_flipped).as_euler('xyz', degrees=True)
                        if rpy.shape == (3,) and not np.isnan(rpy).any():
                            tvec_flat = tvec.flatten()
                            pose = {
                                'tvec': tvec_flat,
                                'rpy': rpy
                            }
                        else:
                            self.get_logger().warn(f":x: RPY 계산 실패 또는 NaN 발생: {rpy}")
            except Exception as e:
                self.get_logger().warn(f":x: YOLO/solvePnP 실패: {e}")
            if pose is None or 'rpy' not in pose or len(pose['rpy']) != 3:
                self.get_logger().warn(f":x: pose 추정 실패 또는 포맷 오류: {pose}")
                return response
            # :white_check_mark: 응답 메시지 작성
            response.x, response.y, response.z = pose['tvec']
            response.rx, response.ry, response.rz = pose['rpy']
            self.get_logger().info(f":outbox_tray: {robot_id} pose 응답 전송 완료")
        # :pushpin: cmd 4~6: 객체 탐지 결과 응답
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
                self.get_logger().info(f":mag_right: {robot_id} - {label} 존재 여부: {found}")
            except Exception as e:
                self.get_logger().warn(f":x: {label} 탐지 실패: {e}")
        return response
    
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

def main(args=None):
    rclpy.init(args=args)
    node = CookGPTServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# # imshow 수신 확인용 
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
