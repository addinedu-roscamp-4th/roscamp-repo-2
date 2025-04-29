"""
[ PGM ]
W           99
H           100

[ YAML ]
Origin  X   -1.26
Origin  Y   -2.13
Resolution  0.05

[ CAM ]
W           1280
H           720
"""


def map_to_webcam(x_map, y_map, origin_x=-1.26, origin_y=-2.13, resolution=0.05):
    """
    맵 좌표 (x_map, y_map) → 웹캠 좌표 (u, v) 로 변환
    기준점:
      - origin_x = -1.26
      - origin_y = -2.13
      - resolution = 0.05 (m/pixel)
    """
    # 맵 좌표 (x_map, y_map)를 현실 좌표 (x_real, y_real)로 변환
    x_real = origin_x + (x_map * resolution)
    y_real = origin_y + (y_map * resolution)

    # 현실 좌표 (x_real, y_real)를 웹캠 좌표 (u, v)로 변환
    u = (x_real + 0.185) / 0.00249 + 240
    v = (y_real - 0.295) / 0.00304 + 109

    return int(round(u)), int(round(v))

def real_to_webcam(x_real, y_real):
    """
    현실 좌표 (x_real, y_real) → 웹캠 좌표 (u, v) 로 변환
    기준점:
      - x_real = -0.185 → u = 240
      - x_real = 1.865 → u = 1063
      - y_real = 0.295 → v = 109
      - y_real = 1.545 → v = 520
    """
    scale_x = 0.00249   # ≈ 2.05 / 823
    scale_y = 0.00304   # ≈ 1.25 / 411

    u = (x_real + 0.185) / scale_x + 240
    v = (y_real - 0.295) / scale_y + 109

    return int(round(u)), int(round(v))

def webcam_to_map(u, v, origin_x=-1.26, origin_y=-2.13, resolution=0.05):
    """
    웹캠 좌표 (u, v) → 맵 좌표 (x_map, y_map) 로 변환
    기준점:
      - origin_x = -1.26
      - origin_y = -2.13
      - resolution = 0.05 (m/pixel)
    """
    # 웹캠 좌표 (u, v)를 먼저 현실 좌표 (x_real, y_real)로 변환
    x_real, y_real = webcam_to_real(u, v)

    # 현실 좌표 (x_real, y_real)를 맵 좌표 (x_map, y_map)로 변환
    x_map = (x_real - origin_x) / resolution
    y_map = (y_real - origin_y) / resolution

    return x_map, y_map

def webcam_to_real(u, v):
    """
    웹캠 좌표 (u, v) → 현실 좌표 (x, y) 로 변환
    기준점:
      - u 기준: 240 → x = -0.185
      - u 기준: 1063 → x = 1.865
      - v 기준: 109 → y = 0.295
      - v 기준: 520 → y = 1.545
    """
    scale_x = 2.05 / 823   # ≈ 0.00249
    scale_y = 1.25 / 411   # ≈ 0.00304

    x_real = scale_x * (u - 240) - 0.185
    y_real = scale_y * (v - 109) + 0.295

    return x_real, y_real
