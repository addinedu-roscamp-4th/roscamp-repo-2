import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import numpy as np
from dataclasses import dataclass

@dataclass
class MycobotCoords:
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

def to_coords_list(raw):
    return [MycobotCoords(*pt) for pt in raw]

def distance(p1, p2):
    return np.linalg.norm(np.array([p1.x, p1.y, p1.z]) - np.array([p2.x, p2.y, p2.z]))

def draw_pose_axes(ax, pose, scale=20.0):
    origin = np.array([pose.x, pose.y, pose.z])
    rot = R.from_euler('xyz', [pose.rx, pose.ry, pose.rz], degrees=True)
    rot_matrix = rot.as_matrix()
    colors = ['r', 'g', 'b']
    for i in range(3):
        vec = rot_matrix[:, i] * scale
        ax.quiver(origin[0], origin[1], origin[2], vec[0], vec[1], vec[2],
                  color=colors[i], length=scale, normalize=True)

def catmull_rom_spline(p0, p1, p2, p3, t):
    return (
        0.5 * (
            2 * p1 +
            (-p0 + p2) * t +
            (2*p0 - 5*p1 + 4*p2 - p3) * t**2 +
            (-p0 + 3*p1 - 3*p2 + p3) * t**3
        )
    )

def interpolate_cr(coords_list, num_points_per_segment=10):
    if len(coords_list) < 4:
        return coords_list
    extended = [coords_list[0]] + coords_list + [coords_list[-1]]
    path = []
    for i in range(1, len(extended) - 2):
        p0, p1, p2, p3 = extended[i-1:i+3]
        for j in range(num_points_per_segment):
            t = j / num_points_per_segment
            interp = MycobotCoords(
                x=catmull_rom_spline(p0.x, p1.x, p2.x, p3.x, t),
                y=catmull_rom_spline(p0.y, p1.y, p2.y, p3.y, t),
                z=catmull_rom_spline(p0.z, p1.z, p2.z, p3.z, t),
                rx=catmull_rom_spline(p0.rx, p1.rx, p2.rx, p3.rx, t),
                ry=catmull_rom_spline(p0.ry, p1.ry, p2.ry, p3.ry, t),
                rz=catmull_rom_spline(p0.rz, p1.rz, p2.rz, p3.rz, t),
            )
            path.append(interp)
    path.append(coords_list[-1])
    return path

def apply_avoidance(path1, path2, threshold=50.0, offset=30.0):
    new_path1 = []
    new_path2 = []
    for idx, (p1, p2) in enumerate(zip(path1, path2)):
        d = distance(p1, p2)
        if d < threshold:
            print(f"[⚠️] 충돌 거리 감지: {d:.2f}mm at idx {idx} → 회피 동작 삽입")
            avoid1 = MycobotCoords(p1.x + offset, p1.y, p1.z, p1.rx, p1.ry, p1.rz)
            avoid2 = MycobotCoords(p2.x - offset, p2.y, p2.z, p2.rx, p2.ry, p2.rz)
            new_path1.append(avoid1)
            new_path2.append(avoid2)
        new_path1.append(p1)
        new_path2.append(p2)
    return new_path1, new_path2

# ✅ 테스트용 via point (고의 충돌 삽입)
traj_dict = {
    'grip_dish_L': [
        [-130, -220, 200, 180, 0, 135],
        [-100, -180, 250, 180, 0, 135],
        [10, -200, 180, 180, 0, 135],
        [10, -200, 230, 180, 0, 135],
        [-40, -130, 328.0, -140, -9, -167],
        [0, 0, 250, 0, 0, 0]
    ],
    'test_traj': [
        [-100, 220, 200, 180, 0, 135],
        [-100, 180, 250, 180, 0, 135],
        [10, 200, 180, 180, 0, 135],
        [10, 200, 230, 180, 0, 135],
        [-40, 100, 328.0, -140, -9, -167],
        [10, 0, 250, 0, 0, 0]
    ]
}

# 변환 및 보간
coords1 = to_coords_list(traj_dict['grip_dish_L'])
coords2 = to_coords_list(traj_dict['test_traj'])
original_path1 = interpolate_cr(coords1)
original_path2 = interpolate_cr(coords2)
avoid_path1, avoid_path2 = apply_avoidance(original_path1, original_path2)

# ✅ 플로팅
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 원래 경로 (얇고 점선)
ax.plot([p.x for p in original_path1], [p.y for p in original_path1], [p.z for p in original_path1],
        label='grip_dish_L original', color='blue', alpha=0.3, linestyle='--')
ax.plot([p.x for p in original_path2], [p.y for p in original_path2], [p.z for p in original_path2],
        label='test_traj original', color='orange', alpha=0.3, linestyle='--')

# 회피된 경로 (굵고 빨간색)
ax.plot([p.x for p in avoid_path1], [p.y for p in avoid_path1], [p.z for p in avoid_path1],
        label='grip_dish_L (avoided)', color='red')
ax.plot([p.x for p in avoid_path2], [p.y for p in avoid_path2], [p.z for p in avoid_path2],
        label='test_traj (avoided)', color='red')

# 원래 viapoint는 별표
for name, coords in zip(traj_dict.keys(), [coords1, coords2]):
    ax.scatter([p.x for p in coords], [p.y for p in coords], [p.z for p in coords],
               marker='*', s=100, label=f'{name} viapoints', color='black')

# 자세축도 함께 표시
for path in [avoid_path1, avoid_path2]:
    for i in range(0, len(path), max(1, len(path)//10)):
        draw_pose_axes(ax, path[i], scale=20.0)

ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_zlabel("Z (mm)")
ax.set_title("Original vs Avoided Trajectories")
ax.legend()
plt.tight_layout()
plt.show()
