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
    print(f"[DEBUG] 로드된 키: {traj_dict.keys()}")

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

        self.timer = self.create_timer(5.0, self.timer_callback)
        self.cook_state_check = self.create_publisher(String, '/cook_state', 10)
        # 각 로봇 토픽 설정
        for ns in self.robot_list:
            self.create_subscription(MycobotCoords, f'{ns}/coords_real', self.make_coords_callback(ns), 10)
            self.coords_pubs[ns] = self.create_publisher(MycobotCoords, f'{ns}/coords_targ', 10)


    def make_coords_callback(self, ns):
        def callback(msg):
            self.latest_coords[ns] = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]
        return callback
    
    def timer_callback(self):
        self.get_logger().info('🕒 5초마다 실행 중...')
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

    
        return
        

def main(args=None):
    rclpy.init(args=args)
    node = PoseBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()