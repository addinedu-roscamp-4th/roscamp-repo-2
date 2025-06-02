import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mycobot_interfaces.msg import MycobotCoords, RobodineCoords
from utils.transform_utils import pose6d_to_homogeneous, homogeneous_to_pose6d
from planning.interpolator import interpolate_linear, interpolate_catmull_rom, interpolate_quintic, compute_adaptive_velocities
from planning.avoidance import is_too_close, compute_avoidance_pose
import yaml, time, threading, os, sys
from ament_index_python.packages import get_package_share_directory

class PoseBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_broadcaster')
        self.robot_list = ['robot48', 'robotb4']
        self.coords_pubs = {}
        self.latest_coords = {}
        self.traj_dict = self.load_trajectory()
        self.menu = 0
        self.cook_sign = False

        self.create_timer(10.0, self.timer_callback)        
        self.cook_state_check = self.create_publisher(String, '/cook_state', 10)

        for ns in self.robot_list:
            self.create_subscription(MycobotCoords, f'{ns}/coords_real', self.make_coords_callback(ns), 10)
            self.coords_pubs[ns] = self.create_publisher(RobodineCoords, f'{ns}/coords_targ', 10)

    def load_trajectory(self):
        pkg_path = get_package_share_directory('cookM')
        yaml_path = os.path.join(pkg_path, 'trajectories.yaml')
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)

    def make_coords_callback(self, ns):
        def callback(msg):
            self.latest_coords[ns] = msg
        return callback

    def timer_callback(self):
        self.get_logger().info('🕒 Timer triggered')
        self.cook_motion_planning()

    def execute_trajectory(self, robot_id: str, trajectory_name: str, mode: int):
        raw_list = self.traj_dict[trajectory_name]
        coords_list = [MycobotCoords(x=float(p[0]), y=float(p[1]), z=float(p[2]),
                                    rx=float(p[3]), ry=float(p[4]), rz=float(p[5])) for p in raw_list]

        # 보간 방식 선택
        path = interpolate_quintic(coords_list)
        # ⏩ 속도 자동 계산
        speeds = compute_adaptive_velocities(path, min_vel=30, max_vel=80)

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

                if is_too_close(100.0):
                    self.get_logger().warn(f"[{robot_id}] 충돌 위험 감지! 회피 기동 중")
                    avoid_pose = compute_avoidance_pose(pose, other_pose, avoid_dist=0.05)

                    remaining = len(path) - idx
                    step = 5 if remaining > 5 else 3 if remaining > 3 else 1

                    next_pose = path[idx + step] if idx + step < len(path) else path[-1]
                    rejoin_path = interpolate_catmull_rom([avoid_pose, next_pose], 5)

                    for p in rejoin_path:
                        send_msg = RobodineCoords(
                            x=p.x, y=p.y, z=p.z,
                            rx=p.rx, ry=p.ry, rz=p.rz,
                            gripper=True,
                            vel=40
                        )
                        self.coords_pubs[robot_id].publish(send_msg)
                        time.sleep(0.1)

                    skip_until = idx + step
                    continue

            send_msg = RobodineCoords(
                x=pose.x, y=pose.y, z=pose.z,
                rx=pose.rx, ry=pose.ry, rz=pose.rz,
                gripper=True,
                vel=speeds[idx]
            )
            self.coords_pubs[robot_id].publish(send_msg)
            time.sleep(0.5 if idx == 1 else 0.1)

        self.get_logger().info(f"✅ {robot_id}의 {trajectory_name} 완료")

    def cook_motion_planning(self):
        if self.menu == 0:
            self.cook_sign = False
            self.cook_state_check.publish(String(data='SETTING'))

            threads = [
                threading.Thread(target=self.execute_trajectory, args=('robot48', 'grip_dish_L', 1)),
                threading.Thread(target=self.execute_trajectory, args=('robotb4', 'grip_dish_R', 1))
            ]
            for t in threads: t.start()
            for t in threads: t.join()

            self.cook_state_check.publish(String(data='COOKING'))


def main(args=None):
    rclpy.init(args=args)
    node = PoseBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
