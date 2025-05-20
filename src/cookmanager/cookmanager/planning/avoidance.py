# 📁 cookmanager/planning/avoidance.py
from mycobot_interfaces.msg import MycobotCoords
from copy import deepcopy
import numpy as np
from math import sqrt
from std_msgs.msg import Float32

def is_too_close(self_pose: MycobotCoords, other_pose: MycobotCoords,robot_id: str, threshold: float = 150.0, distance_publisher=None) -> bool:
    
    if robot_id == 'robot48':
        adjusted_y = other_pose.y - 590
    elif robot_id == 'robotb4':
        adjusted_y = other_pose.y + 590
    else:
        adjusted_y = other_pose.y  # 예외 처리

    dx = self_pose.x - other_pose.x
    dy = self_pose.y - adjusted_y
    dz = self_pose.z - other_pose.z

    distance = sqrt(dx**2 + dy**2 + dz**2)

    # 퍼블리셔가 있으면 거리 퍼블리시
    if distance_publisher is not None:
        msg = Float32()
        msg.data = distance
        distance_publisher.publish(msg)

    return distance < threshold

def compute_avoidance_pose(self_pose: MycobotCoords, other_pose: MycobotCoords,robot_id: str, avoid_dist=50.0) -> MycobotCoords:
    adjusted_other = deepcopy(other_pose)

    if robot_id == 'robot48':
        adjusted_other.y -= 590
    elif robot_id == 'robotb4':
        adjusted_other.y += 590

    new_pose = deepcopy(self_pose)
    dir_vec = np.array([
        self_pose.x - adjusted_other.x,
        self_pose.y - adjusted_other.y,
        self_pose.z - adjusted_other.z
    ])
    norm = np.linalg.norm(dir_vec)

    if norm < 1e-6:
        offset = np.array([0.0, avoid_dist, 0.0])
    else:
        offset = (dir_vec / norm) * avoid_dist

    new_pose.x += offset[0]
    new_pose.y += offset[1]
    new_pose.z += offset[2]
    return new_pose
