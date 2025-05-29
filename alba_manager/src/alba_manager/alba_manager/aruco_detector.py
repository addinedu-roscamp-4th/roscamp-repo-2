import rclpy
from rclpy.node import Node
from alba_msgs.msg import AlbabotCoordinate
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
import tf_transformations
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from collections import deque

import cv2
import numpy as np
import pickle
import time
import math
import json
import socket
import argparse
import struct

from alba_manager.global_cam_config import *
from alba_manager.mapping import *

from ament_index_python.packages import get_package_share_directory
import os

pkg_path = get_package_share_directory('alba_manager')
file_path = os.path.join(pkg_path, 'camera_calibration.pkl')

window_name = 'ArUco Marker Detection'
cv2.namedWindow(window_name)

"""
# 마우스 좌표를 저장하기 위한 전역 변수
current_x, current_y = 0, 0

# 마우스 콜백 함수 정의
def mouse_callback(event, x, y, flags, param):
    global current_x, current_y

    # 마우스 이동 이벤트가 발생하면 좌표를 업데이트
    if event == cv2.EVENT_MOUSEMOVE:
        current_x, current_y = x, y
cv2.setMouseCallback(window_name, mouse_callback)
"""

# 로봇별 색상 지정 (BGR)
ROBOT_COLORS = {
    1: (0, 0, 255),   # 빨강
    2: (0, 255, 0),   # 초록
    3: (255, 0, 0)    # 파랑
}

class SendMapPosition(Node):
    def __init__(self):
        super().__init__('send_map_position')
        self.robot_ids = [1, 2, 3]
        self.robot_paths = {}  # {robot_id: path}
        self.domain_to_robot_id = {
            74: 1,
            62: 2,
            58: 3
        }
        parser = argparse.ArgumentParser()
        parser.add_argument('--host',    default='192.168.0.156', help='Destination IP')
        parser.add_argument('--port',    type=int, default=5000, help='Destination port')
        parser.add_argument('--width',   type=int, default=640,   help='Capture width')
        parser.add_argument('--height',  type=int, default=480,   help='Capture height')
        parser.add_argument('--fps',     type=int, default=20,    help='Frames per second')
        parser.add_argument('--quality', type=int, default=80,    help='JPEG quality (0-100)')
        self.args, unknown = parser.parse_known_args()


        # UDP 소켓 초기화
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 칼만 필터 초기화
        self.kalman_filter_params = {}
        for robot_id in self.robot_ids:
            self.kalman_filter_params[robot_id] = {
                'x': np.array([[0.0], [0.0]]),  # [angle, angle_dot] (2x1)
                'P': np.array([[1.0, 0.0], [0.0, 1.0]]),  # 초기 공분산 (2x2)
                'A': np.array([[1.0, 0.1], [0.0, 1.0]]),  # 상태 전이 모델 (2x2) (dt = 0.1 가정)
                'H': np.array([[1.0, 0.0]]),  # 측정 모델 (1x2)
                'Q': np.array([[0.01, 0.0], [0.0, 0.01]]), # 프로세스 노이즈 공분산 (2x2)
                'R': np.array([[0.1]])   # 측정 노이즈 공분산 (1x1)
            }

        for robot_id in self.robot_ids:
            self.create_subscription(
                Float32,
                f'/albabot{robot_id}/pinky_battery_present',
                lambda msg, robot_id=robot_id: self.battery_callback(robot_id, msg),
                10
            )
            self.create_subscription(
                PoseStamped,
                f'/albabot{robot_id}/tracked_pose',
                lambda msg, robot_id=robot_id: self.pose_callback(robot_id, msg),
                10
            )

        self.subscription = self.create_subscription(
                    String,
                    '/robot_command',
                    self.listener_callback,
                    10)

        self.subscription = self.create_subscription(
                String,
                '/command',
                self.response_callback,
                10)

        self.pub_pos = self.create_publisher(AlbabotCoordinate, '/albabot_pos', 10)
        self.initialpose_publisher = {
            robot_id: self.create_publisher(PoseWithCovarianceStamped, f'albabot{robot_id}/initialpose', 10)
            for robot_id in self.robot_ids
        }
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_topic_publisher = {
            robot_id: self.create_publisher(Odometry, f'albabot{robot_id}/odom', 10)
            for robot_id in self.robot_ids
        }

        self.g_initialpose_msg = {
            robot_id: PoseWithCovarianceStamped()
            for robot_id in self.robot_ids
        }

        self.g_odom_tf_msg = {
            robot_id: TransformStamped()
            for robot_id in self.robot_ids
        }

        self.g_odom_to_base_tf_msg = {
            robot_id: TransformStamped()
            for robot_id in self.robot_ids
        }

        self.g_odom_topic_msg = {
            robot_id: Odometry()
            for robot_id in self.robot_ids
        }

        self.battery = {robot_id: 0.0 for robot_id in self.robot_ids}
        self.euler_history = {robot_id: deque(maxlen=10) for robot_id in self.robot_ids}
        self.last_position = {robot_id: None for robot_id in self.robot_ids}
        self.last_time = {robot_id: None for robot_id in self.robot_ids}
        self.last_orientation = {robot_id: None for robot_id in self.robot_ids}
        self.prev_yaw = {robot_id: None for robot_id in self.robot_ids}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.prev_time = 0

        #self.get_logger().info(pkg_path, file_path)
        try:
            with open(file_path, 'rb') as f:
                calibration_data = pickle.load(f)
            self.get_logger().info("Calibration data loaded successfully")
        except FileNotFoundError:
            self.get_logger().error("Error: Camera calibration file not found")
            return
        except Exception as e:
            self.get_logger().error(f"Error loading calibration data: {e}")
            return

        # 캘리브레이션 데이터 추출
        self.camera_matrix = calibration_data['camera_matrix']
        self.dist_coeffs = calibration_data['dist_coeffs']

        # ArUco 검출기 설정
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        aruco_params = cv2.aruco.DetectorParameters()
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.pub_pos_timer = self.create_timer(0.05, self.timer_callback)

        # 마커 크기 및 3D 좌표 설정 (미터 단위)
        self.marker_size = MARKER_SIZE_MM  # 35mm == 0.35

        # 카메라 설정
        self.cap = cv2.VideoCapture(CAM_DEVICE)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open camera.")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
        #self.print_operation_info()

        # 카메라 초기화 대기
        while not self.cap.isOpened():
            self.get_logger().info("Waiting for the camera to initialize...")

        time.sleep(2)

    def print_operation_info(self):
        self.get_logger().info(f"Camera Device: {CAM_DEVICE}")
        self.get_logger().info(f"Camera Resolution: {CAM_W}x{CAM_H}")
        self.get_logger().info(f"Marker Size: {self.marker_size} m")
        self.get_logger().info("Press 'q' to quit, 'i' for initial pose")

    def battery_callback(self, robot_id, msg):
        if robot_id in self.robot_ids:
            self.battery[robot_id] = msg.data

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        try:
            msg_dict = json.loads(msg.data)
            domain_id = msg_dict.get('domain_id', None)
            robot_id = self.domain_to_robot_id.get(domain_id)
            path = msg_dict.get('path', None)
            if path is not None:
                self.robot_paths[robot_id] = path
                self.get_logger().info(f'Path received: {path}')
            #else:
                #self.get_logger().warn('No path found in the message')
        except json.JSONDecodeError:
            self.get_logger().error('Failed to decode JSON from message')

    def response_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        try:
            # 수신한 JSON 문자열을 파이썬 객체로 변환
            json_data = json.loads(msg.data)
            domain_id = json_data['domain_id']
            command = json_data['command']
            command_status = json_data['command_status']
            robot_id = self.domain_to_robot_id.get(domain_id)
            #self.get_logger().info(f'robot_id {robot_id} command_status: {command_status}')
            if robot_id is not None:
                if command_status == "COMPLETED" or command_status == "EXECUTED":
                    print(self.robot_paths)
                    if robot_id in self.robot_paths:
                        #self.get_logger().info(f'Path{robot_id} release')
                        del self.robot_paths[robot_id]
        except json.JSONDecodeError:
            self.get_logger().error('Failed to decode JSON from message')

    def pose_callback(self, robot_id, msg):
        pass

    def send_frame_via_udp(self, frame):
        interval = 1.0 / self.args.fps
        t0 = time.time()
        # 지정된 해상도로 리사이즈
        frame = cv2.resize(frame, (self.args.width, self.args.height))
        # JPEG로 인코딩
        ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.args.quality])
        if not ok:
            self.get_logger().warn("! JPEG encode failed")
            return
        ts = time.time()
        # 바이트로 변환
        #pkt = buf.tobytes()
        pkt = struct.pack('d', ts) + buf.tobytes()
        self.udp_socket.sendto(pkt, (self.args.host, self.args.port))
        """
        # FPS 유지
        elapsed = time.time() - t0
        if elapsed < interval:
            time.sleep(interval - elapsed)
        """

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def kalman_filter(self, robot_id, z_k):
        """ 칼만 필터 예측 및 업데이트 단계 """
        params = self.kalman_filter_params[robot_id]
        x_k_1, P_k_1, A, H, Q, R = params['x'], params['P'], params['A'], params['H'], params['Q'], params['R']
        # 1. 예측 단계
        x_k_pred = np.dot(A, x_k_1)
        # 각도 변화율 제한
        max_rate = 1.0  # 최대 각도 변화율 (rad/s)
        x_k_pred[1, 0] = np.clip(x_k_pred[1, 0], -max_rate, max_rate)
        P_k_pred = np.dot(np.dot(A, P_k_1), A.T) + Q
        # 2. 업데이트 단계
        y_k = z_k - np.dot(H, x_k_pred)
        S_k = np.dot(np.dot(H, P_k_pred), H.T) + R
        K_k = np.dot(np.dot(P_k_pred, H.T), np.linalg.inv(S_k))
        x_k = x_k_pred + np.dot(K_k, y_k)
        P_k = np.dot((np.eye(2) - np.dot(K_k, H)), P_k_pred)
        # 상태 업데이트
        params['x'], params['P'] = x_k, P_k
        return x_k[0, 0]  # 추정된 각도 반환

    def save_initial_pose(self, robot_id, x, y, yaw):
        # Header 설정
        self.g_initialpose_msg[robot_id].header.frame_id = 'map'
        self.g_initialpose_msg[robot_id].header.stamp = self.get_clock().now().to_msg()
        # Position 설정
        self.g_initialpose_msg[robot_id].pose.pose.position.x = x
        self.g_initialpose_msg[robot_id].pose.pose.position.y = y
        self.g_initialpose_msg[robot_id].pose.pose.position.z = 0.0
        # Orientation 설정 (yaw 값으로부터 quaternion 계산)
        initial_pose_quat = quaternion_from_euler(0.0, 0.0, np.deg2rad(yaw))
        self.g_initialpose_msg[robot_id].pose.pose.orientation.x = initial_pose_quat[0]
        self.g_initialpose_msg[robot_id].pose.pose.orientation.y = initial_pose_quat[1]
        self.g_initialpose_msg[robot_id].pose.pose.orientation.z = initial_pose_quat[2]
        self.g_initialpose_msg[robot_id].pose.pose.orientation.w = initial_pose_quat[3]

        # Covariance 설정 # RVIZ2 2D Pose Estimate에서 사용
        self.g_initialpose_msg[robot_id].pose.covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.06853891909122467
        ]

    def save_odom_topic(self, robot_id, msg):
        self.g_odom_topic_msg[robot_id] = msg

    def save_odom_tf(self, robot_id, msg):
        self.g_odom_tf_msg[robot_id] = msg

    def save_odom_to_base_tf(self, robot_id, msg):
        self.g_odom_to_base_tf_msg[robot_id] = msg

    def is_message_valid(self, msg):
        """주어진 메시지가 발행하기에 유효한지 판단합니다."""
        if msg is None:
            return False

        if hasattr(msg.header, 'stamp') and msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return False # timestamp 0을 유효하지 않다고 간주할 경우

        return True

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            #self.get_logger().warn("Failed to grab frame")
            return

        # 이미지 왜곡 보정
        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        # 마커 검출
        corners, ids, rejected = self.detector.detectMarkers(frame_undistorted)

        # UDP로 프레임 전송
        self.send_frame_via_udp(frame_undistorted)

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
                robot_id = ids[i][0]    # marker id는 1, 2, 3으로 사용하며 robot id도 동일
                if robot_id == 0:
                    robot_id = 1
                if robot_id not in self.robot_ids:
                    continue
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

                self.euler_history[robot_id].append(euler_angles)
                median_euler = np.median(np.array(self.euler_history[robot_id]), axis=0)

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

                    rz = -rz  # z축 회전 각도 반전
                    current_yaw = rz

                    # 칼만 필터 적용
                    kf_yaw = self.kalman_filter(robot_id, np.array([[current_yaw]]))

                    # 위치 및 회전 정보 표시
                    map_pos_x, map_pos_y = webcam_to_map(center_x, center_y)
                    real_x, real_y = map_to_real(map_pos_x, map_pos_y)

                    # 화면에 표시할 정보
                    disp_info = (
                        f"ALBA{robot_id} "
                        f"({map_pos_x:>2.0f}, {map_pos_y:>2.0f}) "
                        f"{kf_yaw:>6.2f} "
                        f"{self.battery[robot_id]:>5.1f}%"
                    )
                    cv2.putText(
                        frame_undistorted,
                        disp_info,
                        (center_x, center_y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (203, 192, 255),
                        2
                    )

                    msg = AlbabotCoordinate()
                    msg.robot_id = int(robot_id)
                    msg.global_pose.position.x = float(round(center_x))
                    msg.global_pose.position.y = float(round(center_y))
                    msg.global_pose.position.z = 0.0
                    msg.global_pose.orientation.x = rx
                    msg.global_pose.orientation.y = ry
                    msg.global_pose.orientation.z = kf_yaw
                    msg.global_pose.orientation.w = 1.0

                    msg.world_pose.position.x = float(round(map_pos_x))
                    msg.world_pose.position.y = float(round(map_pos_y))
                    msg.world_pose.position.z = 0.0
                    msg.world_pose.orientation.x = rx
                    msg.world_pose.orientation.y = ry
                    msg.world_pose.orientation.z = kf_yaw
                    msg.global_pose.orientation.w = 1.0
                    self.pub_pos.publish(msg)

                    self.save_initial_pose(robot_id, real_x, real_y, kf_yaw)

                    #################
                    # Odom TF Publish
                    #################
                    map_to_odom = TransformStamped()
                    map_to_odom.header.stamp = self.get_clock().now().to_msg()
                    map_to_odom.header.frame_id = 'map'
                    map_to_odom.child_frame_id = f'albabot{robot_id}/odom'
                    #map_to_odom.child_frame_id = 'odom'
                    map_to_odom.transform.translation.x = float(real_x)
                    map_to_odom.transform.translation.y = float(real_y)
                    map_to_odom.transform.translation.z = 0.0
                    map_to_odom_quat = quaternion_from_euler(0.0, 0.0, np.deg2rad(kf_yaw))
                    map_to_odom.transform.rotation.x = map_to_odom_quat[0]
                    map_to_odom.transform.rotation.y = map_to_odom_quat[1]
                    map_to_odom.transform.rotation.z = map_to_odom_quat[2]
                    map_to_odom.transform.rotation.w = map_to_odom_quat[3]
                    #self.save_odom_tf(robot_id, map_to_odom)
                    self.tf_broadcaster.sendTransform(map_to_odom)

                    odom_to_base_footprint = TransformStamped()
                    odom_to_base_footprint.header.stamp = self.get_clock().now().to_msg()
                    odom_to_base_footprint.header.frame_id = f'albabot{robot_id}/odom'
                    odom_to_base_footprint.child_frame_id = f'albabot{robot_id}/base_footprint'
                    #odom_to_base_footprint.header.frame_id = 'odom'
                    #odom_to_base_footprint.child_frame_id = 'base_footprint'
                    odom_to_base_footprint.transform.translation.x = 0.0
                    odom_to_base_footprint.transform.translation.y = 0.0
                    odom_to_base_footprint.transform.translation.z = 0.0
                    odom_to_base_footprint_quat = quaternion_from_euler(0.0, 0.0, 0.0)
                    odom_to_base_footprint.transform.rotation.x = odom_to_base_footprint_quat[0]
                    odom_to_base_footprint.transform.rotation.y = odom_to_base_footprint_quat[1]
                    odom_to_base_footprint.transform.rotation.z = odom_to_base_footprint_quat[2]
                    odom_to_base_footprint.transform.rotation.w = odom_to_base_footprint_quat[3]
                    #self.save_odom_to_base_tf(robot_id, odom_to_base_footprint)
                    self.tf_broadcaster.sendTransform(odom_to_base_footprint)

                    curr_transform = map_to_odom.transform
                    curr_quat = map_to_odom_quat

                    #################
                    # Odom Topic Publish
                    #################
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = f'albabot{robot_id}/odom'
                    odom_msg.child_frame_id = f'albabot{robot_id}/base_footprint'
                    #odom_msg.header.frame_id = 'odom'
                    #odom_msg.child_frame_id = 'base_footprint'
                    odom_msg.pose.pose.position.x = curr_transform.translation.x
                    odom_msg.pose.pose.position.y = curr_transform.translation.y
                    odom_msg.pose.pose.position.z = 0.0
                    odom_msg.pose.pose.orientation.x = curr_quat[0]
                    odom_msg.pose.pose.orientation.y = curr_quat[1]
                    odom_msg.pose.pose.orientation.z = curr_quat[2]
                    odom_msg.pose.pose.orientation.w = curr_quat[3]

                    # 위치 공분산 행렬
                    odom_msg.pose.covariance = [
                        0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # y
                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  # z
                        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # roll
                        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  # pitch
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1   # yaw
                    ]

                    # 속도 공분산 행렬
                    odom_msg.twist.covariance = [
                        0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # vx
                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # vy
                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  # vz
                        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # vroll
                        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  # vpitch
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1   # vyaw
                    ]

                    # 현재 위치와 방향
                    current_position = np.array([real_x, real_y, 0.0])
                    current_orientation = odom_msg.pose.pose.orientation
                    # Quaternion을 numpy 배열로 변환
                    current_orientation_np = np.array([
                        current_orientation.x,
                        current_orientation.y,
                        current_orientation.z,
                        current_orientation.w
                    ])

                    last_position = self.last_position[robot_id]
                    last_orientation = self.last_orientation[robot_id]
                    last_time = self.last_time[robot_id]
                    current_time = self.get_clock().now().to_msg()
                    current_time_sec = current_time.sec + current_time.nanosec * 1e-9
                    if (last_position is not None and
                        last_orientation is not None and
                        last_time is not None):
                        # 시간 차이 계산
                        dt = current_time_sec - last_time
                        if dt > 0:
                            # 선속도 계산
                            linear_velocity = (current_position - last_position) / dt

                            # 각속도 계산 (Quaternion 차이)
                            last_orientation_np = np.array([
                                last_orientation.x,
                                last_orientation.y,
                                last_orientation.z,
                                last_orientation.w
                            ])
                            angular_diff = current_orientation_np - last_orientation_np
                            angular_diff = np.arctan2(np.sin(angular_diff), np.cos(angular_diff))
                            angular_velocity = angular_diff / dt

                            # 속도 정보 설정
                            odom_msg.twist.twist.linear.x = float(linear_velocity[0])
                            odom_msg.twist.twist.linear.y = float(linear_velocity[1])
                            odom_msg.twist.twist.linear.z = float(linear_velocity[2])
                            odom_msg.twist.twist.angular.x = float(angular_velocity[0])
                            odom_msg.twist.twist.angular.y = float(angular_velocity[1])
                            odom_msg.twist.twist.angular.z = float(angular_velocity[2])
                        else:
                            # dt가 0이거나 음수인 경우 속도를 0으로 설정
                            odom_msg.twist.twist.linear.x = 0.0
                            odom_msg.twist.twist.linear.y = 0.0
                            odom_msg.twist.twist.linear.z = 0.0
                            odom_msg.twist.twist.angular.x = 0.0
                            odom_msg.twist.twist.angular.y = 0.0
                            odom_msg.twist.twist.angular.z = 0.0
                    else:
                        # 첫 번째 메시지인 경우 속도를 0으로 설정
                        odom_msg.twist.twist.linear.x = 0.0
                        odom_msg.twist.twist.linear.y = 0.0
                        odom_msg.twist.twist.linear.z = 0.0
                        odom_msg.twist.twist.angular.x = 0.0
                        odom_msg.twist.twist.angular.y = 0.0
                        odom_msg.twist.twist.angular.z = 0.0

                    # 현재 위치와 시간 저장
                    self.last_position[robot_id] = current_position
                    self.last_orientation[robot_id] = current_orientation
                    self.last_time[robot_id] = current_time_sec

                    self.save_odom_topic(robot_id, odom_msg)

        for rid, path in self.robot_paths.items():
            color = ROBOT_COLORS.get(rid, (255, 255, 255))  # 미지정시 흰색
            for i, pt in enumerate(path):
                x, y = int(pt[0]), int(pt[1])
                x, y = map_to_webcam(x, y)
                x, y = int(x), int(y)
                if i == 0:
                    # 시작점: 삼각형 마커
                    cv2.drawMarker(frame_undistorted, (x, y), color, markerType=cv2.MARKER_TRIANGLE_UP,
                                   markerSize=10, thickness=2)
                elif i == len(path) - 1:
                    # 끝점: 별 마커
                    cv2.drawMarker(frame_undistorted, (x, y), color, markerType=cv2.MARKER_STAR,
                                   markerSize=10, thickness=2)
                else:
                    # 일반 경로점: 원
                    cv2.circle(frame_undistorted, (int(x), int(y)), 5, color, -1)

            # 로봇 번호 표기 (path의 첫 지점에)
            if path:
                x, y = int(path[0][0]), int(path[0][1])
                #print(x, y)
                x, y = map_to_webcam(x, y)
                x_, y_ = int(x), int(y)
                #print(x_, y_)
                cv2.putText(frame_undistorted, f'R{rid}', (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                #cv2.putText(frame_undistorted, f'R{rid}', (300+8, 300-8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 화면 표시
        cv2.imshow(window_name, frame_undistorted)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.destroy_node()
            rclpy.shutdown()
            pass
        elif key == ord('i'):
            for robot_id, pose_msg in self.g_initialpose_msg.items():
                if self.is_message_valid(pose_msg):
                    self.get_logger().info(f"Publishing Initial Pose: {robot_id} {pose_msg}")
                    # robot_id에 따른 퍼블리셔 사용
                    publisher = self.initialpose_publisher.get(robot_id)
                    if publisher:
                        publisher.publish(pose_msg)
                    else:
                        self.get_logger().warn(f"No publisher found for robot_id: {robot_id}")

        for robot_id, msg in self.g_odom_tf_msg.items():
            if self.is_message_valid(msg):
                #msg.header.stamp = self.get_clock().now().to_msg()
                self.tf_broadcaster.sendTransform(msg)

        for robot_id, msg in self.g_odom_to_base_tf_msg.items():
            if self.is_message_valid(msg):
                #msg.header.stamp = self.get_clock().now().to_msg()
                self.tf_broadcaster.sendTransform(msg)

        for robot_id, msg in self.g_odom_topic_msg.items():
            if self.is_message_valid(msg):
                #msg.header.stamp = self.get_clock().now().to_msg()
                publisher = self.odom_topic_publisher.get(robot_id)
                if publisher:
                    publisher.publish(msg)
                else:
                    self.get_logger().warn(f"No publisher found for robot_id: {robot_id}")




    def destroy_node(self):
        self.cap.release()
        self.udp_socket.close()
        cv2.destroyAllWindows()
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
