import rclpy
from rclpy.node import Node
from alba_msgs.msg import AlbabotCoordinate

import cv2
import numpy as np
import pickle
import time

from alba_manager.global_cam_config import *
from alba_manager.mapping import *

from ament_index_python.packages import get_package_share_directory
import os

pkg_path = get_package_share_directory('alba_manager')
file_path = os.path.join(pkg_path, 'camera_calibration.pkl')

#print(pkg_path, file_path)

class SendMapPosition(Node):
    def __init__(self):
        super().__init__('send_map_position')

        try:
            with open(file_path, 'rb') as f:
                calibration_data = pickle.load(f)
            print("Calibration data loaded successfully")
        except FileNotFoundError:
            print("Error: Camera calibration file not found")
            return
        except Exception as e:
            print(f"Error loading calibration data: {e}")
            return

        # 캘리브레이션 데이터 추출
        self.camera_matrix = calibration_data['camera_matrix']
        self.dist_coeffs = calibration_data['dist_coeffs']

        # ArUco 검출기 설정
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        aruco_params = cv2.aruco.DetectorParameters()
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.pub_pos = self.create_publisher(AlbabotCoordinate, '/albabot_pos', 10)
        self.pub_pos_timer = self.create_timer(0.1, self.timer_callback)

        # 마커 크기 및 3D 좌표 설정 (미터 단위)
        marker_size = MARKER_SIZE_MM  # 35mm == 0.35
        self.marker_3d_edges = np.array([
            [0, 0, 0],
            [0, marker_size, 0],
            [marker_size, marker_size, 0],
            [marker_size, 0, 0]
        ], dtype='float32').reshape((4, 1, 3))

        # 카메라 설정
        self.cap = cv2.VideoCapture(CAM_DEVICE)
        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
        print_operation_info()

        # 카메라 초기화 대기
        while not self.cap.isOpened():
            self.get_logger().info("Waiting for the camera to initialize...")

        time.sleep(2)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to grab frame")
            return


        # 이미지 왜곡 보정
        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        # 마커 검출
        corners, ids, rejected = self.detector.detectMarkers(frame_undistorted)

        # 마커가 검출되면 표시 및 포즈 추정
        if corners is not None and len(corners) > 0:
            for corner, id in zip(corners, ids):
                # 코너 포인트 추출 및 표시
                corner = np.array(corner).reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corner

                center_x = int((topLeft[0] + bottomRight[0]) / 2)
                center_y = int((topLeft[1] + bottomRight[1]) / 2)

                # 코너 포인트 좌표 변환
                topRightPoint = (int(topRight[0]), int(topRight[1]))
                topLeftPoint = (int(topLeft[0]), int(topLeft[1]))
                bottomRightPoint = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeftPoint = (int(bottomLeft[0]), int(bottomLeft[1]))

                # PnP로 포즈 추정
                ret, rvec, tvec = cv2.solvePnP(
                    self.marker_3d_edges,
                    corner.reshape((4, 1, 2)),  # reshape을 2D로
                    self.camera_matrix,
                    self.dist_coeffs
                )

                if ret:
                    # 위치 및 회전 정보 계산
                    x = round(tvec[0][0], 2)
                    y = round(tvec[1][0], 2)
                    z = round(tvec[2][0], 2)
                    rx = round(np.rad2deg(rvec[0][0]), 2)
                    ry = round(np.rad2deg(rvec[1][0]), 2)
                    rz = round(np.rad2deg(rvec[2][0]), 2)

                    # 위치 및 회전 정보 표시
                    map_pos_x, map_pos_y = webcam_to_map(center_x, center_y)
                    #print(map_pos_x, map_pos_y)
                    msg = AlbabotCoordinate()
                    msg.robot_id = int(id)
                    msg.global_pose.position.x = float(round(center_x))
                    msg.global_pose.position.y = float(round(center_y))
                    msg.global_pose.position.z = 0.0
                    msg.global_pose.orientation.x = rx
                    msg.global_pose.orientation.y = ry
                    msg.global_pose.orientation.z = rz
                    msg.global_pose.orientation.w = 1.0

                    msg.world_pose.position.x = float(round(map_pos_x))
                    msg.world_pose.position.y = float(round(map_pos_y))
                    msg.world_pose.position.z = 0.0
                    msg.world_pose.orientation.x = rx
                    msg.world_pose.orientation.y = ry
                    msg.world_pose.orientation.z = rz
                    msg.world_pose.orientation.w = 1.0
                    #print(msg)
                    self.pub_pos.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SendMapPosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

