import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mycobot_interfaces.msg import MycobotCoords
from mycobot_interfaces.srv import CookGPTsrv
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
import time
import threading
import os
from ament_index_python.packages import get_package_share_directory
# cookgpt에서 좌표값이 제대로 인식이 안되는데, 시각화를 잘못한 건지 진짜 solvePnP를 잘못추정한건지 확인필요
# ✅ trajectories.yaml 경로 확보
pkg_path = get_package_share_directory('cookM')
yaml_path = os.path.join(pkg_path, 'trajectories.yaml')

with open(yaml_path, 'r') as f:
    traj_dict = yaml.safe_load(f)
    print(f"[DEBUG] 로드된 키: {traj_dict.keys()}")

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

class PoseBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_broadcaster')
        self.robot_list = ['robot48', 'robotb4']
        self.angles_pubs = {}
        self.coords_pubs = {}
        self.latest_angles = {}
        self.latest_coords = {}
        self.traj_dict = traj_dict
        self.menu = 0
        self.cook_sign = False
        self.pickup_sign = True
        # self.cook_state_check = ""

        self.timer = self.create_timer(10.0, self.timer_callback)
        self.cook_state_check = self.create_publisher(String, '/cook_state', 10)
        # 각 로봇 토픽 설정
        for ns in self.robot_list:
            self.create_subscription(MycobotCoords, f'{ns}/coords_real', self.make_coords_callback(ns), 10)
            self.coords_pubs[ns] = self.create_publisher(MycobotCoords, f'{ns}/coords_targ', 10)

        self.cli = self.create_client(CookGPTsrv, 'CookGPTsrv')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('🔄 cook_pose_service 대기 중...')

        self.req = CookGPTsrv.Request()


    def make_coords_callback(self, ns):
        def callback(msg):
            self.latest_coords[ns] = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]
        return callback
    
    def timer_callback(self):
        self.get_logger().info('🕒 10초마다 실행 중...')
        self.cook_motion_planning()     
    
    def execute_trajectory(self, robot_id, trajectory_name):
        for pose in self.traj_dict[trajectory_name]:
            msg = MycobotCoords()
            msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = [float(v) for v in pose]
            self.coords_pubs[robot_id].publish(msg)
            time.sleep(3)
        self.get_logger().info(f"✅ {robot_id}의 {trajectory_name} 완료")

    def cook_motion_planning(self):
        # 샐러드 모션 플래닝
        # menu 주문 들어오면 수행

        self.cook_sign = False # 다음 menu_callback에서 cook_motion_planning 또 실행되는 것 방지
            # 1st, place the plate
            #       
        command = 'SETTING'
        msg = String()
        msg.data = command # pick up 가능
        self.cook_state_check.publish(msg)
            # trajectory 동시 실행
        threads = []

        t1 = threading.Thread(target=self.execute_trajectory, args=('robot48', 'grip_dish_L'))
        t2 = threading.Thread(target=self.execute_trajectory, args=('robotb4', 'test_traj'))

        threads.append(t1)
        threads.append(t2)

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # salad motion planning
        for robot_id in self.robot_list:
            self.req.command = 0
            self.req.robot_id = robot_id 
            response = self.cli.call(self.req)
            self.handle_pose_response(response, robot_id) # If return value is true -> okay //
                    
    
        return
    
    def calculate_grip_pose(self, ns, T_cam_2_obj):
        T_ee_2_cam = [35.355, -35.355, 45, 0, 0, 45]
        T_ee_2_gripper = [14, -14, 100.0, 0, 0, 45]

        T_ee = pose6d_to_homogeneous(self.latest_coords[ns])
        T_ee_2_cam_ = pose6d_to_homogeneous(T_ee_2_cam)
        T_gripper_2_ee = np.linalg.inv(pose6d_to_homogeneous(T_ee_2_gripper))
        T_cam_2_obj_ = pose6d_to_homogeneous(T_cam_2_obj)

        T_base_2_target = T_ee @ T_ee_2_cam_ @ T_cam_2_obj_
        T_base_2_ee_target = T_base_2_target @ T_gripper_2_ee
        pose = homogeneous_to_pose6d(T_base_2_ee_target)
        pose[2] += 20.0
        return pose
    
    def handle_pose_response(self, response, robot_id):
        try:
            pose = [response.x, response.y, response.z,
            response.rx, response.ry, response.rz]

            grip_pose = self.calculate_grip_pose(robot_id, pose)

            msg = MycobotCoords()
            msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = grip_pose
            self.coords_pubs[robot_id].publish(msg)

            self.get_logger().info(f"✅ {robot_id}에게 grip pose publish 완료")

        except Exception as e:
            self.get_logger().warn(f"❌ grip pose 처리 실패: {e}")
        

def main(args=None):
    print("🔥 [DEBUG] cook_comp_test main() 진입됨", flush=True)
    rclpy.init(args=args)
    node = PoseBroadcaster() 
    rclpy.spin(node)
    rclpy.shutdown()


# if __name__ == '__main__':
#     main()