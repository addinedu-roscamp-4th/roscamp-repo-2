# 📁 cookmanager/utils/transform_utils.py
import numpy as np
from scipy.spatial.transform import Rotation as R

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
