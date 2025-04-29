import os
import sys

CAM_DEVICE = "/dev/global_cam"
#CAM_W = 1920
#CAM_H = 1080
CAM_W = 1280
CAM_H = 720
#CAM_W = 640
#CAM_H = 480
CHECKER_BLOCK_SIZE_MM = 0.02
MARKER_SIZE_MM = 0.042

def print_operation_info():
    if os.path.exists(CAM_DEVICE):
        print(f"({CAM_W} x {CAM_H}) checker_size: {CHECKER_BLOCK_SIZE_MM} mm, marker_size: {MARKER_SIZE_MM} mm")
    else:
        print(f"{CAM_DEVICE} not found!")
        sys.exit(1)
