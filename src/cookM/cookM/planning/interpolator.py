# 📁 cookM/planning/interpolator.py
from mycobot_interfaces.msg import MycobotCoords
from scipy.interpolate import CubicSpline
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

def interpolate_linear(coords_list, num_points_per_segment=20):
    if len(coords_list) < 2:
        return coords_list

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
    return path

def interpolate_cubic(coords_list, num_points_per_segment=10):
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

def interpolate_catmull_rom(coords_list, num_points_per_segment=20):
    def catmull_rom(p0, p1, p2, p3, t):
        return 0.5 * (2*p1 + (-p0 + p2)*t + (2*p0 - 5*p1 + 4*p2 - p3)*t**2 + (-p0 + 3*p1 - 3*p2 + p3)*t**3)

    n = len(coords_list)
    if n < 4:
        return interpolate_cubic(coords_list, num_points_per_segment)

    extended = [coords_list[0]] + coords_list + [coords_list[-1]]
    path = []
    for i in range(1, len(extended) - 2):
        p0, p1, p2, p3 = extended[i-1], extended[i], extended[i+1], extended[i+2]
        for j in range(num_points_per_segment):
            t = j / num_points_per_segment
            interp = MycobotCoords()
            interp.x = float(catmull_rom(p0.x, p1.x, p2.x, p3.x, t))
            interp.y = float(catmull_rom(p0.y, p1.y, p2.y, p3.y, t))
            interp.z = float(catmull_rom(p0.z, p1.z, p2.z, p3.z, t))
            interp.rx = float(catmull_rom(p0.rx, p1.rx, p2.rx, p3.rx, t))
            interp.ry = float(catmull_rom(p0.ry, p1.ry, p2.ry, p3.ry, t))
            interp.rz = float(catmull_rom(p0.rz, p1.rz, p2.rz, p3.rz, t))
            path.append(interp)
    path.append(coords_list[-1])
    return path

def to_quaternion_list(coords_list):
    return R.from_euler('xyz', [[p.rx, p.ry, p.rz] for p in coords_list], degrees=True)


def interpolate_quaternion_path(coords_list, num_points=100):
    if len(coords_list) < 2:
        return coords_list

    positions = np.linspace(0, len(coords_list)-1, len(coords_list))
    slerp_times = np.linspace(0, len(coords_list)-1, num_points)
    rots = to_quaternion_list(coords_list)
    slerp = Slerp(positions, rots)
    interp_rots = slerp(slerp_times)
    interp_euler = interp_rots.as_euler('xyz', degrees=True)

    xs = np.linspace(coords_list[0].x, coords_list[-1].x, num_points)
    ys = np.linspace(coords_list[0].y, coords_list[-1].y, num_points)
    zs = np.linspace(coords_list[0].z, coords_list[-1].z, num_points)

    result = []
    for i in range(num_points):
        p = MycobotCoords()
        p.x, p.y, p.z = xs[i], ys[i], zs[i]
        p.rx, p.ry, p.rz = interp_euler[i]
        result.append(p)
    return result


def generate_quintic_coeff(p0, p1, T):
    # Quintic 보간 계수 계산 (속도, 가속도 0)
    a0 = p0
    a1 = 0
    a2 = 0
    a3 = (10 * (p1 - p0)) / T**3
    a4 = (-15 * (p1 - p0)) / T**4
    a5 = (6 * (p1 - p0)) / T**5
    return a0, a1, a2, a3, a4, a5

def interpolate_quintic_with_timing(coords_list, sampling_time, segment_duration):
    """
    - coords_list: via point 리스트 (MycobotCoords 또는 6D dict)
    - sampling_time: 제어 루프 주기 (예: 0.05초)
    - segment_duration: 각 구간이 걸리는 시간 (초)
    """
    interpolated_path = []
    velocities = []

    for i in range(len(coords_list) - 1):
        p0 = coords_list[i]
        p1 = coords_list[i+1]

        # 각 차원별 계수 계산
        coeffs = {}
        for dim in ['x', 'y', 'z', 'rx', 'ry', 'rz']:
            a0, a1, a2, a3, a4, a5 = generate_quintic_coeff(getattr(p0, dim), getattr(p1, dim), segment_duration)
            coeffs[dim] = (a0, a1, a2, a3, a4, a5)

        # 보간 시간 스텝 수
        n_steps = int(segment_duration / sampling_time)
        for j in range(n_steps):
            t = j * sampling_time
            interp = MycobotCoords()
            for dim in ['x', 'y', 'z', 'rx', 'ry', 'rz']:
                a0, a1, a2, a3, a4, a5 = coeffs[dim]
                val = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
                setattr(interp, dim, val)
            interpolated_path.append(interp)

            # 속도 벡터 norm 계산 (xyz 기준)
            vx = 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
            vy = 3*coeffs['y'][3]*t**2 + 4*coeffs['y'][4]*t**3 + 5*coeffs['y'][5]*t**4
            vz = 3*coeffs['z'][3]*t**2 + 4*coeffs['z'][4]*t**3 + 5*coeffs['z'][5]*t**4
            v_norm = np.sqrt(vx**2 + vy**2 + vz**2)
            velocities.append(v_norm)

    # 속도 벡터 → 0~50 사이로 정규화
    v_min = min(velocities)
    v_max = max(velocities)
    v_mapped = [int(np.clip((v - v_min) / (v_max - v_min + 1e-6) * 50, 25, 50)) for v in velocities]

    return interpolated_path, v_mapped


def interpolate_quintic(coords_list, num_points_per_segment=20):
    def quintic_poly(p0, p1, t):
        return p0 + (p1 - p0) * (10 * t**3 - 15 * t**4 + 6 * t**5)

    path = []
    for i in range(len(coords_list) - 2):  # ⚠️ 마지막 포인트 바로 전까지만 보간
        p0 = coords_list[i]
        p1 = coords_list[i + 1]
        for j in range(num_points_per_segment):
            t = j / num_points_per_segment
            p = MycobotCoords()
            p.x = quintic_poly(p0.x, p1.x, t)
            p.y = quintic_poly(p0.y, p1.y, t)
            p.z = quintic_poly(p0.z, p1.z, t)
            p.rx = quintic_poly(p0.rx, p1.rx, t)
            p.ry = quintic_poly(p0.ry, p1.ry, t)
            p.rz = quintic_poly(p0.rz, p1.rz, t)
            path.append(p)

    # 마지막 보간 없이 딱 붙이기
    path.append(coords_list[-2])  # 마지막 보간 구간 시작점
    path.append(coords_list[-1])  # 마지막 점 그대로 추가
    path.append(coords_list[-1])  # 명령 씹힐경우 대비 한 번 더
    return path

def compute_adaptive_velocities(path, min_vel=10, max_vel=90):
    """
    인접 포인트 간 거리를 기반으로 속도를 조절하여 부드러운 경로 수행을 가능하게 함.
    - min_vel: 가장 가까운 거리일 때의 속도
    - max_vel: 가장 먼 거리일 때의 속도
    """
    if len(path) < 2:
        return [min_vel] * len(path)

    # 거리 리스트 계산
    distances = []
    for i in range(len(path) - 1):
        dx = path[i+1].x - path[i].x
        dy = path[i+1].y - path[i].y
        dz = path[i+1].z - path[i].z
        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        distances.append(dist)

    # 거리 정규화 (0~1)
    d_min = min(distances)
    d_max = max(distances)
    norm_d = [(d - d_min) / (d_max - d_min + 1e-6) for d in distances]  # 0 division 방지

    # 속도 계산
    velocities = [int(min_vel + (max_vel - min_vel) * d) for d in norm_d]
    velocities.append(velocities[-1])  # 마지막 포인트도 동일한 속도로

    return velocities