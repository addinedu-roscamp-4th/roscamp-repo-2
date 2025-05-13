import cv2 as cv
import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor
import matplotlib.pyplot as plt



def detect_box_coordinates_orb_with_scene(template_path, kp_scene, des_scene, scene_img_gray):
    template = cv.imread(template_path, cv.IMREAD_GRAYSCALE)
    if template is None or des_scene is None:
        print(f"템플릿 로딩 실패: {template_path}")
        return None

    orb = cv.ORB_create(nfeatures=500 , scaleFactor=1.2, nlevels=4)
    kp_template, des_template = orb.detectAndCompute(template, None)
    if des_template is None:
        return None

    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des_template, des_scene)
    if len(matches) < 4:
        return None

    matches = sorted(matches, key=lambda x: x.distance)
    min_dist = matches[0].distance
    good = [m for m in matches if m.distance < 3 * min_dist]

    if len(good) >= 4:
        src_pts = np.float32([kp_template[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp_scene[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)
        if M is not None:
            h, w = template.shape
            box = np.float32([[0,0], [0,h-1], [w-1,h-1], [w-1,0]]).reshape(-1, 1, 2)
            dst = cv.perspectiveTransform(box, M)
            return dst
    return None

#메인 실행
scene_color = cv.imread('20250429_145138.jpg')
scene_gray = cv.cvtColor(scene_color, cv.COLOR_BGR2GRAY)

#scene 이미지에서 keypoints 한 번만 계산
orb = cv.ORB_create(nfeatures=500 , scaleFactor=1.2, nlevels=4)
kp_scene, des_scene = orb.detectAndCompute(scene_gray, None)

start = time.perf_counter()

#병렬 처리로 두 템플릿 매칭
with ThreadPoolExecutor(max_workers=2) as executor:
    future_A = executor.submit(detect_box_coordinates_orb_with_scene, '20250429_162839.jpg', kp_scene, des_scene, scene_gray)
    future_B = executor.submit(detect_box_coordinates_orb_with_scene, '20250429_162938.jpg', kp_scene, des_scene, scene_gray)
    dst_A = future_A.result()
    dst_B = future_B.result()

end = time.perf_counter()
print(f"\nORB 병렬 검출 시간: {end - start:.4f}초")

# 시각화
if dst_A is not None:
    scene_color = cv.polylines(scene_color, [np.int32(dst_A)], True, (0,255,0), 5, cv.LINE_AA)
if dst_B is not None:
    scene_color = cv.polylines(scene_color, [np.int32(dst_B)], True, (255,0,0), 5, cv.LINE_AA)

# plt.figure(figsize=(12, 8))
# plt.imshow(cv.cvtColor(scene_color, cv.COLOR_BGR2RGB))
# plt.axis("off")
# plt.title("ORB: A(녹색), B(파랑) 인식 결과")
# plt.show()
