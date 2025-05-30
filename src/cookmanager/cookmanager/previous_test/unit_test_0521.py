import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from mycobot_interfaces.srv import CookGPTsrv
from mycobot_interfaces.msg import MycobotCoords, RobodineCoords
from cookmanager.utils.transform_utils import pose6d_to_homogeneous, homogeneous_to_pose6d
from cookmanager.planning.interpolator import interpolate_linear, interpolate_catmull_rom, interpolate_quintic_with_timing
from cookmanager.planning.avoidance import is_too_close, compute_avoidance_pose
import yaml, time, threading, os, sys
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
import numpy as np

class PoseBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_broadcaster')
        self.robot_list = ['robot48', 'robotb4']
        self.coords_pubs = {}
        self.latest_coords = {}
        self.trajectories = {}
        self.traj_dict = self.load_trajectory()
        self.menu = 2
        self.cook_sign = False
        self.emergency_flag = False

        self.timer = self.create_timer(3.0, self.timer_callback)  
        self.cook_state_check = self.create_publisher(String, '/cook_state', 10)
        self.distance_pub = self.create_publisher(Float32, '/distance', 10)

        for ns in self.robot_list:
            self.create_subscription(Bool, f'{ns}/hand_detected', self.emergency_stop(ns), 10)
            self.create_subscription(MycobotCoords, f'{ns}/coords_real', self.make_coords_callback(ns), 10)
            self.coords_pubs[ns] = self.create_publisher(RobodineCoords, f'{ns}/coords_targ', 10)

        

        # self.cli = self.create_client(CookGPTsrv, '/CookGPTsrv')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('🔄 cook_pose_service 대기 중...')

        # self.req = CookGPTsrv.Request()


    def load_trajectory(self):
        pkg_path = get_package_share_directory('cookmanager')
        yaml_path = os.path.join(pkg_path, 'trajectories.yaml')
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)

    def make_coords_callback(self, ns):
        def callback(msg):
            self.latest_coords[ns] = msg
        return callback
    
    def emergency_stop(self, ns):   
        def callback(msg):
            self.emergency_flag[ns] = msg
        return callback

    def timer_callback(self):
        self.get_logger().info('🕒 Timer triggered')

        self.timer.cancel()  # ✅ 한 번만 실행하고 멈추게 함

        self.cook_motion_planning()

    def synchronized_execute(self, sampling_time, grip_timing, release_timing):
        robot_ids = list(self.trajectories.keys())
        paths = [self.trajectories[r][0] for r in robot_ids]
        speeds = [self.trajectories[r][1] for r in robot_ids]

        path_len = min(len(p) for p in paths)  # 둘 중 짧은 경로 기준

        for idx in range(path_len):
            for i, robot_id in enumerate(robot_ids):
                pose = paths[i][idx]
                speed = speeds[i][idx]

                if idx <= grip_timing:
                    gripper_vel = 80
                elif idx > release_timing:
                    gripper_vel = 80
                else:
                    gripper_vel = 0

                send_msg = RobodineCoords(
                    x=pose.x, y=pose.y, z=pose.z,
                    rx=pose.rx, ry=pose.ry, rz=pose.rz,
                    gripper=gripper_vel,
                    vel=speed
                )
                self.coords_pubs[robot_id].publish(send_msg)

            if idx == 0:
                time.sleep(2.0)
            else:
                time.sleep(sampling_time)  # 동기 타이밍 유지

    def execute_trajectory(self, robot_id: str, trajectory_name: str, grip_timing: int, release_timing: int, segment_duration: float, sampling_time: float, grip_state: bool):
        if grip_state:
            gripper_vel = 0
        else:
            gripper_vel = 80
        
        interpol_num = segment_duration/sampling_time
        raw_list = self.traj_dict[trajectory_name]
        coords_list = [MycobotCoords(x=float(p[0]), y=float(p[1]), z=float(p[2]),
                                    rx=float(p[3]), ry=float(p[4]), rz=float(p[5])) for p in raw_list]

        # 보간 방식 선택
        path, speeds = interpolate_quintic_with_timing(coords_list, sampling_time, segment_duration)

        if robot_id == "robotb4":
            time.sleep(segment_duration)

        skip_until = 0
        for idx, pose in enumerate(path):
            if idx < skip_until:
                continue

            # 충돌 판단용 다른 로봇들의 현재 pose 평균
            other_coords = [self.latest_coords[r] for r in self.robot_list if r != robot_id and r in self.latest_coords]
            if other_coords:
                mx = sum(p.x for p in other_coords) / len(other_coords)
                my = sum(p.y for p in other_coords) / len(other_coords)
                mz = sum(p.z for p in other_coords) / len(other_coords)
                mrx = sum(p.rx for p in other_coords) / len(other_coords)
                mry = sum(p.ry for p in other_coords) / len(other_coords)
                mrz = sum(p.rz for p in other_coords) / len(other_coords)

                other_pose = MycobotCoords(x=mx, y=my, z=mz, rx=mrx, ry=mry, rz=mrz)
                # self.get_logger().warn(f"b4의 좌표: {other_pose} // 48의 좌표: {pose}")

                if is_too_close(pose, other_pose, robot_id, 10, distance_publisher=self.distance_pub):
                    self.get_logger().warn(f"[{robot_id}] 충돌 위험 감지! 회피 기동 중")
                    avoid_pose = compute_avoidance_pose(pose, other_pose,robot_id, avoid_dist=30)

                    #회피 우선 실행
                    send_msg = RobodineCoords(
                            x=avoid_pose.x, y=avoid_pose.y, z=avoid_pose.z,
                            rx=avoid_pose.rx, ry=avoid_pose.ry, rz=avoid_pose.rz,
                            gripper=gripper_vel,
                            vel=40
                        )
                    self.coords_pubs[robot_id].publish(send_msg)
                    time.sleep(sampling_time*3)

                    remaining = len(path) - idx
                    step = 7 if remaining > 7 else 5 if remaining > 5 else 3 if remaining > 3 else 1

                    next_pose = path[idx + step] if idx + step < len(path) else path[-1]
                    rejoin_path = interpolate_linear([avoid_pose, next_pose], 5)

                    for p in rejoin_path:
                        send_msg = RobodineCoords(
                            x=p.x, y=p.y, z=p.z,
                            rx=p.rx, ry=p.ry, rz=p.rz,
                            gripper=gripper_vel,
                            vel=40
                        )
                        self.coords_pubs[robot_id].publish(send_msg)
                        time.sleep(sampling_time)

                    skip_until = idx + step
                    continue

            
            
            if idx > grip_timing*interpol_num:
                gripper_vel = 0
                if idx > release_timing*interpol_num:
                    gripper_vel = 80
            elif idx > release_timing*interpol_num:
                gripper_vel = 80
                if idx > grip_timing*interpol_num:
                    gripper_vel = 0

            send_msg = RobodineCoords(
                x=pose.x, y=pose.y, z=pose.z,
                rx=pose.rx, ry=pose.ry, rz=pose.rz,
                gripper=gripper_vel,
                vel=speeds[idx]
            )
            self.coords_pubs[robot_id].publish(send_msg)
            if idx == 0:
                time.sleep(2.0)
            else:
                time.sleep(sampling_time)

        self.get_logger().info(f"✅ {robot_id}의 {trajectory_name} 완료")

    def cook_motion_planning(self):

        self.cook_sign = False
        
        # self.get_logger().info("setting 드갑니다") # sync
        # self.cook_state_check.publish(String(data='SETTING'))

        # for robot_id, traj_name in [('robot48', 'grip_dish_L'), ('robotb4', 'grip_dish_R')]:
        #     raw_list = self.traj_dict[traj_name]
        #     sampling_time = 0.3
        #     segment_duration = 6.0
        #     coords_list = [MycobotCoords(x=float(p[0]), y=float(p[1]), z=float(p[2]),
        #                                 rx=float(p[3]), ry=float(p[4]), rz=float(p[5])) for p in raw_list]
        #     path, speeds = interpolate_quintic_with_timing(coords_list, sampling_time, segment_duration)
        #     self.trajectories[robot_id] = (path, speeds)
        
        # self.synchronized_execute(sampling_time, 20, 97) # 그릇 드는 것은 동시에 움직여야 하니까 싱크
        
        
        # time.sleep(1.0)
        self.get_logger().info("salad 드갑니다") # thread
        self.cook_state_check.publish(String(data='COOKING'))

        threads = []
        threads = [
            threading.Thread(target=self.execute_trajectory, args=('robot48', 'grip_ingredient_L', 200, 3, 4.8, 0.3, True)),
            threading.Thread(target=self.execute_trajectory, args=('robotb4', 'grip_ingredient_R', 200, 3, 4.8, 0.3, True))
        ]
        for t in threads: t.start()
        for t in threads: t.join()

        time.sleep(1.0)
        # self.get_logger().info("sauce 드갑니다") # thread
        
        # threads = []
        # threads = [ # 대충 가면서 잡게끔 할라고 25
        #     threading.Thread(target=self.execute_trajectory, args=('robot48', 'grip_sauce_L1', 3.2, 200, 6.0, 0.3, False)), 
        #     threading.Thread(target=self.execute_trajectory, args=('robotb4', 'grip_sauce_R2', 3.7, 200, 6.0, 0.3, False))
        # ]
        # for t in threads: t.start()
        # for t in threads: t.join()

        # time.sleep(1.0)
        # self.get_logger().info("shaking") # thread
        
        # threads = []
        # threads = [ # 대충 가면서 잡게끔 할라고 25
        #     threading.Thread(target=self.execute_trajectory, args=('robot48', 'grip_sauce_Ls', 200, 200, 0.8, 0.4, True)), 
        #     threading.Thread(target=self.execute_trajectory, args=('robotb4', 'grip_sauce_Rs', 200, 200, 0.8, 0.4, True))
        # ]
        # for t in threads: t.start()
        # for t in threads: t.join()

        # time.sleep(1)
        # self.get_logger().info("sauce return") # thread
        
        
        # threads = []
        # threads = [ # 대충 가면서 잡게끔 할라고 25
        #     threading.Thread(target=self.execute_trajectory, args=('robot48', 'grip_sauce_L2', 200, 4.0, 6.0, 0.3, True)), 
        #     threading.Thread(target=self.execute_trajectory, args=('robotb4', 'grip_sauce_R2', 200, 4.0, 6.0, 0.3, True))
        # ]
        # for t in threads: t.start()
        # for t in threads: t.join()

        # # time.sleep(1.0)
        # self.get_logger().info("pickup 드갑니다") # sync  
        # # albabot이 도착했다는 flag가 하나 있어야 할 것 같은데 
        # self.cook_state_check.publish(String(data='PICKUP'))
        
        # for robot_id, traj_name in [('robot48', 'grip_pickup_L'), ('robotb4', 'grip_pickup_R')]:
        #     sampling_time = 0.3
        #     segment_duration = 6.0
        #     raw_list = self.traj_dict[traj_name]
        #     coords_list = [MycobotCoords(x=float(p[0]), y=float(p[1]), z=float(p[2]),
        #                                 rx=float(p[3]), ry=float(p[4]), rz=float(p[5])) for p in raw_list]
        #     path, speeds = interpolate_quintic_with_timing(coords_list, sampling_time, segment_duration)
        #     self.trajectories[robot_id] = (path, speeds)
        
        # self.synchronized_execute(sampling_time, 37, 97) # 그릇 드는 것은 동시에 움직여야 하니까 싱크
        # self.cook_state_check.publish(String(data='IDLE'))

        return

        # match self.menu:
        #     case 0: # dongas motion planning
        #         self.get_logger().info("dongas")
        #     case 1: # pasta motion planning
        #         self.get_logger().info("pasta")
        #     case 2: # salad motion planning
                # self.get_logger().info("salad 드갑니다")
                # for robot_id in self.robot_list:
                #     req = CookGPTsrv.Request()
                #     req.command = 2
                #     req.robot_id = robot_id

                #     self.get_logger().info(f"📡 {robot_id} pose 요청")
                #     future = self.cli.call_async(req)

                #     # ✅ 비동기 콜백 등록
                #     future.add_done_callback(lambda fut, rid=robot_id: self.on_pose_response(fut, rid))

                # 필요한 경우 요리 상태 전환
                # self.cook_state_check.publish(String(data='COOKING'))
                


                # threads = [
                #     threading.Thread(target=self.execute_trajectory, args=('robot48', 'grip_ingredient_L', 1)),
                #     threading.Thread(target=self.execute_trajectory, args=('robotb4', 'grip_ingredient_R', 1))
                # ]
                # for t in threads: t.start()
                # for t in threads: t.join()

                # time.sleep(2)

                    # grip과 release 구분해서 할 지 말 지 고민.. -> 잡았다 신호를 받고 출발해야 하나? 

                # if self.pickup_sign:  # 픽업 센서 있다면, 그거 고려해서 판별식 수정
                #     self.cook_state_check.publish(String(data='PICKUP'))
                #     # 4. release trajectory 실행
                #     release_threads = []
                #     release_threads.append(threading.Thread(target=self.execute_trajectory, args=('robot_48', 'grip_pickup_L')))
                #     release_threads.append(threading.Thread(target=self.execute_trajectory, args=('robot_b4', 'grip_pickup_R')))

                #     for t in threads: t.start()
                #     for t in threads: t.join()

                #     self.cook_state_check.publish(String(data='IDLE'))
                # else:
                #     self.get_logger().warn("⚠️ pickup 실패")
                
        #     case 3: # steak motion planning
        #         print("steak 만들었다")
    
        # return
    
    def on_pose_response(self, future, robot_id):
        try:
            response = future.result()
            self.get_logger().info(f"🧪 {robot_id} 응답 수신: {response}")

            self.handle_pose_response(response, robot_id)
            self.get_logger().info(f"✅ {robot_id}의 pose 처리 완료")

        except Exception as e:
            self.get_logger().warn(f"❌ {robot_id} 응답 처리 실패: {e}")
    
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
        pose[2] += 30.0
        return pose
    
    def handle_pose_response(self, response, robot_id):
        self.get_logger().info(f"📍 handle_pose_response 진입 - {robot_id}")  # 🔍 진입 확인
        try:
            pose = [
                float(response.x), float(response.y), float(response.z),
                float(response.rx), float(response.ry), float(response.rz)
            ]
            self.get_logger().info(f"✅ handle_pose 도착")

            grip_pose = self.calculate_grip_pose(robot_id, pose)
            self.get_logger().info(f"✅ cal_grip_계산")

            msg = RobodineCoords()
            msg.x, msg.y, msg.z = float(grip_pose[0]), float(grip_pose[1]), float(grip_pose[2])
            msg.rx, msg.ry, msg.rz = float(grip_pose[3]), float(grip_pose[4]), float(grip_pose[5])
            msg.gripper = False # 이동
            msg.vel = 30            
            self.coords_pubs[robot_id].publish(msg)
            self.get_logger().info(f"✅ publish 함~")

            time.sleep(1) # 그립 포인트 이동 후

            msg.gripper = True # 그립
            self.coords_pubs[robot_id].publish(msg)

            self.get_logger().info(f"✅ {robot_id}에게 grip pose publish 완료")

        except Exception as e:
            self.get_logger().warn(f"❌ grip pose 처리 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseBroadcaster()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
