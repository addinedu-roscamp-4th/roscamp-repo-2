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
        R_flip = R.from_euler('x', [180], degrees=True).as_matrix()

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
                results = model(imgd, verbose=False)[0]

                pose = None
                if self.get_pose.get(robot_name, False):

                    if results.keypoints is not None and len(results.keypoints) > 0:
                        imgp = results.keypoints[0].xy.cpu().numpy()[:4].astype(np.float32)
                        ret, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs)
                        if ret:
                            R_obj_cam, _ = cv2.Rodrigues(rvec)
                            rpy = R.from_matrix(R_obj_cam).as_euler('xyz', degrees=True)
                            tvec_flat = tvec.flatten()
                            rpy = rpy @ R_flip # 6D 포즈 z축을 바닥으로 두기 위함
                            pose = {
                                'tvec': tvec_flat,
                                'rpy': rpy
                            }
                

                with self.frame_locks[robot_name]:
                    self.latest_frames[robot_name] = imgd
                    self.latest_results[robot_name] = results
                    self.latest_pose[robot_name] = pose

            except socket.timeout:
                continue

    def handle_request(self, request, response):
        robot_id = request.robot_id
        cmd = request.command

        if robot_id not in self.robot_ports:
            self.get_logger().warn(f"❌ 알 수 없는 로봇 ID 요청: {robot_id}")
            return response  # 모든 필드는 default (0, False)로 유지됨

        with self.frame_locks[robot_id]:
            results = self.latest_results.get(robot_id, None)
            pose = self.latest_pose.get(robot_id, None)

        if results is None:
            self.get_logger().warn(f"⚠️ 아직 {robot_id} 프레임 준비 안됨")
            return response

        if cmd in [0, 1, 2, 3]:
            self.get_pose[robot_id] = True
            if pose is None:
                self.get_logger().warn(f"❌ {robot_id} pose 없음")
                return response
            else:
                response.robot_id = robot_id
                response.x, response.y, response.z = pose['tvec']
                response.rx, response.ry, response.rz = pose['rpy']
                self.get_pose = False
                self.get_logger().info(f"📤 {robot_id} pose 응답 전송 완료")

        elif cmd == 4:
            label = "dish"
            found = any([int(r[5]) == model.names.index(label) for r in results.boxes.data.cpu().numpy()])
            response.dish = found
            self.get_logger().info(f"🔎 {robot_id} - dish 존재 여부: {found}")

        elif cmd == 5:
            label = "sauce"
            found = any([int(r[5]) == model.names.index(label) for r in results.boxes.data.cpu().numpy()])
            response.sauce = found
            self.get_logger().info(f"🔎 {robot_id} - sauce 존재 여부: {found}")

        elif cmd == 6:
            label = "stain"
            found = any([int(r[5]) == model.names.index(label) for r in results.boxes.data.cpu().numpy()])
            response.stain = found
            self.get_logger().info(f"🔎 {robot_id} - stain 존재 여부: {found}")

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
