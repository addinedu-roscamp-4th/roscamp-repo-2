import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mycobot_interfaces.msg import MycobotCoords, RobodineCoords
from mycobot_interfaces.srv import CookGPTsrv
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
import time
import threading
import os
from ament_index_python.packages import get_package_share_directory
from copy import deepcopy
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# ✅ trajectories.yaml 경로 확보
pkg_path = get_package_share_directory('cookM')
yaml_path = os.path.join(pkg_path, 'trajectories.yaml')

with open(yaml_path, 'r') as f:
    traj_dict = yaml.safe_load(f)
    print(f"[DEBUG] 로드된 키: {traj_dict.keys()}")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

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
        # self.create_subscription(String,'/menu_item', self.menu_callback, 10)
        self.cook_state_check = self.create_publisher(String, '/cook_state', 10)
        # 각 로봇 토픽 설정
        for ns in self.robot_list:
            self.create_subscription(MycobotCoords, f'{ns}/coords_real', self.make_coords_callback(ns), 10)
            self.coords_pubs[ns] = self.create_publisher(RobodineCoords, f'{ns}/coords_targ', 10)

        # self.cli = self.create_client(CookGPTsrv, 'CookGPTsrv')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('🔄 cook_pose_service 대기 중...')

        # self.req = CookGPTsrv.Request()


    def make_coords_callback(self, ns):
        def callback(msg):
            self.latest_coords[ns] = msg  # ✅ 이제 MycobotCoords 객체를 그대로 저장
        return callback
    
    def timer_callback(self):
        self.get_logger().info('🕒 10초마다 실행 중...')
        self.cook_motion_planning()
    
    def menu_callback(self, msg):
        command = msg.data
        if command == 'dongas':
            self.menu = 0
            self.cook_sign = True
        elif command == 'pasta':
            self.menu = 1
            self.cook_sign = True
        elif command == 'salad':
            self.menu = 2
            self.cook_sign = True
        elif command == 'steak':
            self.menu = 3
            self.cook_sign = True
        else:
            self.cook_sign = False
            return
        if self.cook_sign:
            self.cook_motion_planning()     

    # ✅ 별도로 정의된 fallback cubic 보간 함수
    def cubic_spline_interpolate_path(self, coords_list, num_points_per_segment=10):
        n = len(coords_list)
        if n < 2:
            return coords_list

        t = np.arange(n)
        x = np.array([p.x for p in coords_list])
        y = np.array([p.y for p in coords_list])
        z = np.array([p.z for p in coords_list])
        rx = np.array([p.rx for p in coords_list])
        ry = np.array([p.ry for p in coords_list])
        rz = np.array([p.rz for p in coords_list])

        cs_x = CubicSpline(t, x)
        cs_y = CubicSpline(t, y)
        cs_z = CubicSpline(t, z)
        cs_rx = CubicSpline(t, rx)
        cs_ry = CubicSpline(t, ry)
        cs_rz = CubicSpline(t, rz)

        t_dense = np.linspace(0, n - 1, (n - 1) * num_points_per_segment)
        path = []
        for ti in t_dense:
            p = MycobotCoords()
            p.x = float(cs_x(ti))
            p.y = float(cs_y(ti))
            p.z = float(cs_z(ti))
            p.rx = float(cs_rx(ti))
            p.ry = float(cs_ry(ti))
            p.rz = float(cs_rz(ti))
            path.append(p)

        path.append(coords_list[-1])
        return path
    
    def linear_interpolate_path(self, coords_list, num_points_per_segment=20):
        
        if len(coords_list) < 2:
            return coords_list

        path = []
        for i in range(len(coords_list) - 1):
            start = coords_list[i]
            end = coords_list[i + 1]

            for j in range(num_points_per_segment):
                t = j / num_points_per_segment

                interp = MycobotCoords()
                interp.x = (1 - t) * start.x + t * end.x
                interp.y = (1 - t) * start.y + t * end.y
                interp.z = (1 - t) * start.z + t * end.z
                interp.rx = (1 - t) * start.rx + t * end.rx
                interp.ry = (1 - t) * start.ry + t * end.ry
                interp.rz = (1 - t) * start.rz + t * end.rz

                path.append(interp)
        
        return path


    def catmull_rom_interpolate_path(self, coords_list, num_points_per_segment=20):
        def catmull_rom_spline(p0, p1, p2, p3, t):
            return (
                0.5 * (
                    2 * p1 +
                    (-p0 + p2) * t +
                    (2*p0 - 5*p1 + 4*p2 - p3) * t**2 +
                    (-p0 + 3*p1 - 3*p2 + p3) * t**3
                )
            )

        n = len(coords_list)
        if n < 4:
            return self.cubic_spline_interpolate_path(coords_list, num_points_per_segment)

        # 🟢 endpoint 보완
        extended_coords = [coords_list[0]] + coords_list + [coords_list[-1]]

        path = []
        for i in range(1, len(extended_coords) - 2):
            p0, p1, p2, p3 = extended_coords[i-1], extended_coords[i], extended_coords[i+1], extended_coords[i+2]

            for j in range(num_points_per_segment):
                t = j / num_points_per_segment

                interp = MycobotCoords()
                interp.x = float(catmull_rom_spline(p0.x, p1.x, p2.x, p3.x, t))
                interp.y = float(catmull_rom_spline(p0.y, p1.y, p2.y, p3.y, t))
                interp.z = float(catmull_rom_spline(p0.z, p1.z, p2.z, p3.z, t))
                interp.rx = float(catmull_rom_spline(p0.rx, p1.rx, p2.rx, p3.rx, t))
                interp.ry = float(catmull_rom_spline(p0.ry, p1.ry, p2.ry, p3.ry, t))
                interp.rz = float(catmull_rom_spline(p0.rz, p1.rz, p2.rz, p3.rz, t))

                path.append(interp)

        # 마지막 via point를 명시적으로 추가
        path.append(coords_list[-1])
        return path

    
    def local_avoidance(self_pose: MycobotCoords, other_pose: MycobotCoords, avoid_dist=50.0):
        """
        두 pose가 가까울 경우, self_pose를 other_pose로부터 회피시키기 위한 새로운 pose 생성.
        위치는 XY 평면상에서 반대 방향으로 일정 거리 떨어지게 한다.
        """
        new_pose = deepcopy(self_pose)

        # 방향 벡터 계산
        dir_vec = np.array([
            self_pose.x - other_pose.x,
            self_pose.y - other_pose.y,
            self_pose.z - other_pose.z
        ])
        norm = np.linalg.norm(dir_vec)

        if norm < 1e-6:
            # 완전히 겹쳐있다면 X축 기준으로 회피
            offset = np.array([avoid_dist, 0.0])
        else:
            offset = (dir_vec / norm) * avoid_dist

        # 적용
        new_pose.x += offset[0]
        new_pose.y += offset[1]
        new_pose.z += offset[2]

        return new_pose
    
    # collision_checker.py

    def is_too_close(self, distance: float, threshold: float = 50.0) -> bool:
        return distance < threshold


    
    # 만약 그리퍼 위치 따라서 회피 기동 할라면 여기서 추가해야할듯
    def execute_trajectory(self, robot_id: str, trajectory_name: str, mode):
        # 1. Load via point list from YAML
        raw_list = self.traj_dict[trajectory_name]
        coords_list = []
        cook_mode = mode

        for pt in raw_list:
            coord = MycobotCoords()
            coord.x, coord.y, coord.z, coord.rx, coord.ry, coord.rz = [float(v) for v in pt]
            coords_list.append(coord)

        # 2. Interpolate trajectory
        if cook_mode == 1: # cooking -> 같이 들어야 하니까 interpolation 직선으로
            path = self.linear_interpolate_path(coords_list)
            print("linear!")
        else:
            path = self.catmull_rom_interpolate_path(coords_list)
            print("catmul!")

    

        # 3. For each pose, check for collision and publish
        for idx, pose in enumerate(path):
            # 🔸 충돌 판단용: 다른 로봇들의 pose 평균 계산
            others = [r for r in self.robot_list if r != robot_id]
            valid_poses = [self.latest_coords[r] for r in others if self.latest_coords[r] is not None]

            if valid_poses:
                # 평균 위치/자세 계산
                mx = sum(p.x for p in valid_poses) / len(valid_poses)
                my = sum(p.y for p in valid_poses) / len(valid_poses)
                mz = sum(p.z for p in valid_poses) / len(valid_poses)
                mrx = sum(p.rx for p in valid_poses) / len(valid_poses)
                mry = sum(p.ry for p in valid_poses) / len(valid_poses)
                mrz = sum(p.rz for p in valid_poses) / len(valid_poses)

                # 평균 pose 생성
                other_pose = MycobotCoords()
                other_pose.x, other_pose.y, other_pose.z = mx, my, mz
                other_pose.rx, other_pose.ry, other_pose.rz = mrx, mry, mrz

                # curr_distance = self.current_distance # gripper 인식을 통해 업데이트 할 예정
                curr_distance = 100.0 

                if self.is_too_close(curr_distance):
                    self.get_logger().warn(f"[{robot_id}] 충돌 위험 감지! 회피 기동 중")

                    # 회피 pose 생성
                    avoid_pose = self.local_avoidance(pose, other_pose, avoid_dist=0.05)

                    # 원래 경로의 다음 포즈가 있다면 부드럽게 합류 경로 생성
                    if idx + 5 < len(path):
                        next_pose = path[idx + 5]
                        rejoin_path = self.catmull_rom_interpolate_path([avoid_pose, next_pose],5)

                        # 회피 + 재합류 경로 수행
                        for p in rejoin_path:
                            send_msg = RobodineCoords()
                            send_msg.x, send_msg.y, send_msg.z = p.x, p.y, p.z
                            send_msg.rx, send_msg.ry, send_msg.rz = p.rx, p.ry, p.rz
                            send_msg.gripper = True
                            send_msg.vel = 50
                            self.coords_pubs[robot_id].publish(send_msg)
                            time.sleep(0.1)

                        idx += 5  # 원래 next_pose는 스킵
                        continue
                    elif idx + 3 < len(path):
                        next_pose = path[idx + 3]
                        rejoin_path = self.catmull_rom_interpolate_path([avoid_pose, next_pose],5)

                        # 회피 + 재합류 경로 수행
                        for p in rejoin_path:
                            send_msg = RobodineCoords()
                            send_msg.x, send_msg.y, send_msg.z = p.x, p.y, p.z
                            send_msg.rx, send_msg.ry, send_msg.rz = p.rx, p.ry, p.rz
                            send_msg.gripper = True
                            send_msg.vel = 50
                            self.coords_pubs[robot_id].publish(send_msg)
                            time.sleep(0.1)

                        idx += 3  # 원래 next_pose는 스킵
                        continue
                    elif idx + 1 < len(path):
                        next_pose = path[idx + 1]
                        rejoin_path = self.catmull_rom_interpolate_path([avoid_pose, next_pose],5)

                        # 회피 + 재합류 경로 수행
                        for p in rejoin_path:
                            send_msg = RobodineCoords()
                            send_msg.x, send_msg.y, send_msg.z = p.x, p.y, p.z
                            send_msg.rx, send_msg.ry, send_msg.rz = p.rx, p.ry, p.rz
                            send_msg.gripper = True
                            send_msg.vel = 50
                            self.coords_pubs[robot_id].publish(send_msg)
                            time.sleep(0.1)

                        idx += 1  # 원래 next_pose는 스킵
                        continue

            # 정상 경로 수행
            send_msg = RobodineCoords()
            send_msg.x, send_msg.y, send_msg.z = pose.x, pose.y, pose.z
            send_msg.rx, send_msg.ry, send_msg.rz = pose.rx, pose.ry, pose.rz
            send_msg.gripper = True
            
            if idx==1:
                send_msg.vel = 50
                self.coords_pubs[robot_id].publish(send_msg)            
                time.sleep(0.5)
            else:
                send_msg.vel = 30
                self.coords_pubs[robot_id].publish(send_msg)            
                time.sleep(0.1)

        self.get_logger().info(f"✅ {robot_id}의 {trajectory_name} 완료")

    def cook_motion_planning(self):
        # 샐러드 모션 플래닝
        # menu 주문 들어오면 수행
        # menu = self.menu
        menu = 0
        self.cook_sign = False # 다음 menu_callback에서 cook_motion_planning 또 실행되는 것 방지
            # 1st, place the plate
            #       
        
        if menu == 0:
            command = 'SETTING'
            msg = String()
            msg.data = command # pick up 가능
            self.cook_state_check.publish(msg)
                # trajectory 동시 실행
            threads = []

            t1 = threading.Thread(target=self.execute_trajectory, args=('robot48', 'grip_dish_L', 1))
            t2 = threading.Thread(target=self.execute_trajectory, args=('robotb4', 'grip_dish_R', 1))

            threads.append(t1)
            threads.append(t2)

            for t in threads:
                t.start()
            for t in threads:
                t.join()

            # salad motion planning
            command = 'COOKING'
            msg = String()
            msg.data = command
            self.cook_state_check.publish(msg)

            # for robot_id in self.robot_list:
            #     self.req.command = 0
            #     self.req.robot_id = robot_id 
            #     response = self.cli.call(self.req)
            #     self.handle_pose_response(response, robot_id) # If return value is true -> okay //
        else:
            return
    
        return
    
    def calculate_grip_pose(self, ns, T_cam_2_obj):
        T_ee_2_cam = [35.355, -35.355, 45, 0, 0, 45]
        T_ee_2_gripper = [14, -14, 100.0, 0, 0, 45]

        msg = self.latest_coords[ns]
        pose6d = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]

        T_ee = pose6d_to_homogeneous(pose6d)
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

            msg = RobodineCoords()
            msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = grip_pose
            msg.gripper = False # 이동
            msg.vel = 30            
            self.coords_pubs[robot_id].publish(msg)

            time.sleep(1) # 그립 포인트 이동 후

            msg = RobodineCoords()
            msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = grip_pose
            msg.gripper = True # 그립
            msg.vel = 30            
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


if __name__ == '__main__':
    main()