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
        self.marker_size = MARKER_SIZE_MM  # 35mm == 0.35

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
        if ids is not None:
            # 검출된 마커 표시
            #cv2.aruco.drawDetectedMarkers(frame_undistorted, corners, ids)

            # 각 마커의 포즈 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            # 각 마커에 대해 처리
            for i in range(len(ids)):
                # 코너 포인트 추출 및 표시
                # 좌표축 표시
                """
                cv2.drawFrameAxes(frame_undistorted, self.camera_matrix, self.dist_coeffs,
                                rvecs[i], tvecs[i], self.marker_size/2)
                """

                # 마커의 3D 위치 표시
                pos_x = tvecs[i][0][0]
                pos_y = tvecs[i][0][1]
                pos_z = tvecs[i][0][2]

                # 회전 벡터를 오일러 각도로 변환
                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]
                #euler_angles = [abs(angle) for angle in euler_angles]  # 각도를 절대값으로 변환

                # 마커 정보 표시
                corner = corners[i][0]
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))


                if ret:
                    # 위치 및 회전 정보 계산
                    x = round(pos_x, 2)
                    y = round(pos_y, 2)
                    z = round(pos_z, 2)
                    rx = round(euler_angles[0], 2)
                    ry = round(euler_angles[1], 2)
                    rz = round(euler_angles[2], 2)

                    # 위치 및 회전 정보 표시
                    map_pos_x, map_pos_y = webcam_to_map(center_x, center_y)
                    msg = AlbabotCoordinate()
                    msg.robot_id = int(ids[i][0])
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

