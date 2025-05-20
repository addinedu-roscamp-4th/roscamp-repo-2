import rclpy
from rclpy.node import Node
from mycobot_interfaces.msg import MycobotAngles, MycobotCoords
from std_srvs.srv import Trigger
from mycobot_interfaces.srv import TriggerGrasp
import numpy as np
from scipy.spatial.transform import Rotation as R
import socket
import threading

def pose6d_to_homogeneous(pose6d):
    x, y, z, rx, ry, rz = pose6d
    translation = np.array([x, y, z]) * 0.001
    rotation = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T

def homogeneous_to_pose6d(T):
    translation = T[:3, 3] * 1000.0
    rotation = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    return np.concatenate([translation, rotation])

# 고정된 변환들
T_ee_2_cam = [35.355, -35.355, 45, 0, 0, 45]
T_ee_2_gripper = [14, -14, 100.0, 0, 0, 45]

class M5TeleopCommander(Node):
    def __init__(self):
        super().__init__('m5_teleop_commander')

        self.robot_list = ['robot48', 'robotb4']
        self.get_logger().info(f"🔵 연결할 로봇들: {self.robot_list}")

        self.angles_pubs = {}
        self.coords_pubs = {}
        self.latest_angles = {}
        self.latest_coords = {}

        self.grasp_bool = False
        self.last_cam_obj_pose = None

        for ns in self.robot_list:
            self.create_subscription(MycobotAngles, f'{ns}/angles_real', self.make_angles_callback(ns), 10)
            self.create_subscription(MycobotCoords, f'{ns}/coords_real', self.make_coords_callback(ns), 10)

            self.angles_pubs[ns] = self.create_publisher(MycobotAngles, f'{ns}/angles_targ', 10)
            self.coords_pubs[ns] = self.create_publisher(MycobotCoords, f'{ns}/coords_targ', 10)

        # grasp 요청 서비스 등록
        self.create_service(TriggerGrasp, 'trigger_grasp', self.trigger_grasp_callback)

        # UDP 서버 스레드 시작
        threading.Thread(target=self.start_udp_server, daemon=True).start()

    def make_angles_callback(self, ns):
        def callback(msg):
            self.latest_angles = [msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6]
            # self.get_logger().info(f"🟢 [{ns}] 현재 관절 상태 수신: {self.latest_angles}")
            pass
        return callback

    def make_coords_callback(self, ns):
        def callback(msg):
            self.latest_coords[ns] = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]
            # self.get_logger().info(f"🟢 [{ns}] 현재 관절 상태 수신: {self.latest_coords[ns]}")
        return callback

    def trigger_grasp_callback(self, request, response):
        ns = request.ns

        if ns not in self.robot_list:
            response.success = False
            response.message = f"알 수 없는 로봇 네임스페이스: {ns}"
            return response

        if self.last_cam_obj_pose is None:
            response.success = False
            response.message = "아직 T_cam_2_obj를 수신하지 않았습니다."
            return response

        if ns not in self.latest_coords:
            response.success = False
            response.message = f"{ns}의 현재 pose를 수신하지 못했습니다."
            return response

        self.get_logger().info(f"✅ [{ns}] grasp 요청 수신: pose 계산 및 전송 시작")
        self.process_pose(ns, self.last_cam_obj_pose)
        
        response.success = True
        response.message = f"[{ns}] grasp 동작 완료"
        return response

    def start_udp_server(self, ip='0.0.0.0', port=9000):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, port))
        self.get_logger().info(f"📡 UDP 서버 실행 중: {ip}:{port}")

        while rclpy.ok():
            data, addr = sock.recvfrom(1024)
            try:
                values = list(map(float, data.decode().strip().split(',')))
                if len(values) == 6:
                    self.last_cam_obj_pose = values

                print("성공 : ", values)
            except Exception as e:
                self.get_logger().warn(f"❌ UDP 데이터 파싱 실패: {e}")

    def process_pose(self, ns, T_cam_2_obj):
        T_ee = pose6d_to_homogeneous(self.latest_coords[ns])
        T_ee_2_cam_ = pose6d_to_homogeneous(T_ee_2_cam)
        T_ee_2_gripper_ = pose6d_to_homogeneous(T_ee_2_gripper)
        T_gripper_2_ee = np.linalg.inv(T_ee_2_gripper_)
        T_cam_2_obj_ = pose6d_to_homogeneous(T_cam_2_obj)

        T_base_2_target = T_ee @ T_ee_2_cam_ @ T_cam_2_obj_
        T_base_2_ee_target = T_base_2_target @ T_gripper_2_ee

        # target_z = T_base_2_target[:3, 2]
        # ee_z = T_base_2_ee_target[:3, 2]
        # if np.dot(target_z, ee_z) > 0:
        #     self.get_logger().info("🌀 방향 보정 수행")
        #     R_flip = R.from_euler('zy', [180, 180], degrees=True).as_matrix()
        #     T_base_2_target[:3, :3] = T_base_2_target[:3, :3] @ R_flip
        #     T_base_2_ee_target = T_base_2_target @ T_gripper_2_ee

        T_base_2_ee_target = T_base_2_target @ T_gripper_2_ee

        target_pose = homogeneous_to_pose6d(T_base_2_ee_target)
        target_pose[2] += 20.0 # 2cm 오프셋
        self.get_logger().info(f"🎯 [{ns}] 이동할 최종 pose: {target_pose}")

        msg = MycobotCoords()
        msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = target_pose
        self.coords_pubs[ns].publish(msg)
        print("Target Pose: ", target_pose)

def main(args=None):
    rclpy.init(args=args)
    node = M5TeleopCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

