import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mycobot_interfaces.msg import MycobotCoords, RobodineCoords
from scipy.spatial.transform import Rotation as R
import numpy as np
from scipy.interpolate import CubicSpline
import yaml
import time
import threading
import os
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
from copy import deepcopy

# ✅ Load trajectory YAML
pkg_path = get_package_share_directory('cookM')
yaml_path = os.path.join(pkg_path, 'trajectories.yaml')
with open(yaml_path, 'r') as f:
    traj_dict = yaml.safe_load(f)

# ✅ Prepare figure for path visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

class PoseBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_broadcaster')
        self.robot_list = ['robot48', 'robotb4']
        self.coords_pubs = {}
        self.latest_coords = {}
        self.traj_dict = traj_dict

        self.timer = self.create_timer(10.0, self.timer_callback)
        self.cook_state_check = self.create_publisher(String, '/cook_state', 10)

        for ns in self.robot_list:
            self.create_subscription(MycobotCoords, f'{ns}/coords_real', self.make_coords_callback(ns), 10)
            self.coords_pubs[ns] = self.create_publisher(RobodineCoords, f'{ns}/coords_targ', 10)

    def make_coords_callback(self, ns):
        def callback(msg):
            self.latest_coords[ns] = msg
        return callback

    def timer_callback(self):
        self.get_logger().info('🕒 10초마다 실행 중...')
        self.cook_motion_planning()

    def linear_interpolate_path(self, coords_list, num_points_per_segment=20):
        path = []
        for i in range(len(coords_list) - 1):
            start, end = coords_list[i], coords_list[i + 1]
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
        path.append(coords_list[-1])
        return path

    def get_interpolated_path(self, trajectory_name, mode):
        raw_list = self.traj_dict[trajectory_name]
        coords_list = [MycobotCoords(x=float(p[0]), y=float(p[1]), z=float(p[2]),
                                     rx=float(p[3]), ry=float(p[4]), rz=float(p[5])) for p in raw_list]
        return self.linear_interpolate_path(coords_list) if mode == 1 else self.catmull_rom_interpolate_path(coords_list)

    def catmull_rom_interpolate_path(self, coords_list, num_points_per_segment=20):
        def catmull_rom_spline(p0, p1, p2, p3, t):
            return 0.5 * (2 * p1 + (-p0 + p2) * t + (2*p0 - 5*p1 + 4*p2 - p3) * t**2 + (-p0 + 3*p1 - 3*p2 + p3) * t**3)

        if len(coords_list) < 4:
            return self.linear_interpolate_path(coords_list)

        extended = [coords_list[0]] + coords_list + [coords_list[-1]]
        path = []
        for i in range(1, len(extended) - 2):
            p0, p1, p2, p3 = extended[i - 1], extended[i], extended[i + 1], extended[i + 2]
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
        path.append(coords_list[-1])
        return path

    def run_trajectory(self, robot_id, path):
        for idx, pose in enumerate(path):
            msg = RobodineCoords()
            msg.x, msg.y, msg.z = pose.x, pose.y, pose.z
            msg.rx, msg.ry, msg.rz = pose.rx, pose.ry, pose.rz
            msg.gripper = True
            msg.vel = 50 if idx == 1 else 30
            self.coords_pubs[robot_id].publish(msg)
            time.sleep(0.5 if idx == 1 else 0.1)
        self.get_logger().info(f"✅ {robot_id} 경로 실행 완료")

    def cook_motion_planning(self):
        path_48 = self.get_interpolated_path('grip_dish_L', mode=1)
        path_b4 = self.get_interpolated_path('grip_dish_R', mode=1)

        for name, path in [('robot48', path_48), ('robotb4', path_b4)]:
            xs = [p.x for p in path]
            ys = [p.y for p in path]
            zs = [p.z for p in path]
            ax.plot(xs, ys, zs, label=name)
        ax.set_title("Interpolated Trajectories")
        ax.legend()
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        plt.tight_layout()
        plt.show()
        time.sleep(10)
        t1 = threading.Thread(target=self.run_trajectory, args=('robot48', path_48))
        t2 = threading.Thread(target=self.run_trajectory, args=('robotb4', path_b4))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        self.cook_state_check.publish(String(data='COOKING'))


def main(args=None):
    rclpy.init(args=args)
    node = PoseBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
