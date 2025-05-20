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

# ✅ trajectories.yaml 경로 확보
pkg_path = get_package_share_directory('cookM')
yaml_path = os.path.join(pkg_path, 'trajectories.yaml')

with open(yaml_path, 'r') as f:
    traj_dict = yaml.safe_load(f)

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
        self.menu = 0
        self.cook_sign = False
        self.pickup_sign = True
        self.cook_state_check = ""

        self.create_subscription(String,'/menu_item', self.menu_callback, 10)
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
    
    def menu_callback(self, msg):
        command = msg.data
        if command == 'salad':
            self.menu = '0'
            self.cook_sign = True
        elif command == 'steak':
            self.menu = '1'
            self.cook_sign = True
        elif command == 'pasta':
            self.menu = '2'
            self.cook_sign = True
        elif command == 'dongas':
            self.menu = '3'
            self.cook_sign = True
        else:
            self.cook_sign = False
            return 
        
        if self.cook_sign:
            self.cook_motion_planning(self.menu)     
    
    def execute_trajectory(self, robot_id, trajectory_name):
        for pose in self.traj_dict[trajectory_name]:
            msg = MycobotCoords()
            msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = pose
            self.coords_pubs[robot_id].publish(msg)
            time.sleep(1)
        self.get_logger().info(f"✅ {robot_id}의 {trajectory_name} 완료")

    def cook_motion_planning(self, menu):
        # 샐러드 모션 플래닝
        # menu 주문 들어오면 수행
        menu = self.menu

        self.cook_sign = False # 다음 menu_callback에서 cook_motion_planning 또 실행되는 것 방지
            # 1st, place the plate
        self.req.command = int('4') # find the dish
        for robot_id in self.robot_list:
            self.req.robot_id = robot_id 
            future = self.cli.call_async(self.req)
            future.add_done_callback(self.handle_bool_response) # If return value is true -> okay //
            
        command = 'SETTING'
        msg = String()
        msg.data = command # pick up 가능
        self.cook_state_check.publish(msg)
            # trajectory 동시 실행
        threads = []

        t1 = threading.Thread(target=self.execute_trajectory, args=('robot_48', 'grip_dish_L'))
        t2 = threading.Thread(target=self.execute_trajectory, args=('robot_b4', 'test_traj'))

        threads.append(t1)
        threads.append(t2)

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        command = 'COOKING'
        msg = String()
        msg.data = command # pick up 가능
        self.cook_state_check.publish(msg)

        match menu:
            case '0': # salad motion planning
                for robot_id in self.robot_list:
                    self.req.command = 0
                    self.req.robot_id = robot_id 
                    response = self.cli.call(self.req)
                    self.handle_pose_response(response, robot_id) # If return value is true -> okay //
                    
                # trajectory 양 팔 동시 실행
                threads = []
                # 재료랑 소스 뿌리는 것 까지
                t1 = threading.Thread(target=self.execute_trajectory, args=('robot_48', 'grip_ingredient_L'))
                t2 = threading.Thread(target=self.execute_trajectory, args=('robot_b4', 'grip_ingredient_R'))

                threads.append(t1)
                threads.append(t2)

                for t in threads:
                    t.start()
                for t in threads:
                    t.join()

                    # grip과 release 구분해서 할 지 말 지 고민.. -> 잡았다 신호를 받고 출발해야 하나? 

                if self.pickup_sign:  # 픽업 센서 있다면, 그거 고려해서 판별식 수정
                    command = 'PICKUP'
                    msg = String()
                    msg.data = command # pick up 가능
                    self.cook_state_check.publish(msg)
                    # 4. release trajectory 실행
                    release_threads = []
                    release_threads.append(threading.Thread(target=self.execute_trajectory, args=('robot_48', 'grip_pickup_L')))
                    release_threads.append(threading.Thread(target=self.execute_trajectory, args=('robot_b4', 'grip_pickup_R')))

                    for t in release_threads:
                        t.start()
                    for t in release_threads:
                        t.join()

                    command = 'IDLE'
                    msg = String()
                    msg.data = command # 조리 끝 / 서빙 시작
                    self.cook_state_check.publish(msg)
                else:
                    self.get_logger().warn("⚠️ pickup 실패")

            case '1': # steak motion planning
                print("steak 만들었다")
            case '2': # pasta motion planning
                print("pasta 만들었다")
            case '3': # dongas motion planning
                print("dongas 만들었다")
        
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

    def handle_bool_response(self, future):
        try:
            response = future.result()
            if response.dish:
                result = response.success
                self.get_logger().info(f"🔔 dish 있음: {result}")
            elif response.sauce:
                result = response.success
                self.get_logger().info(f"🔔 sauce 있음: {result}")
            elif response.stain:
                result = response.success
                self.get_logger().info(f"🔔 얼룩 있음: {result}")
            else:
                self.get_logger().info("🔔 오류: ")

            # 여기에 필요한 동작 추가 가능
        except Exception as e:
            self.get_logger().warn(f"❌ bool 응답 처리 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()



