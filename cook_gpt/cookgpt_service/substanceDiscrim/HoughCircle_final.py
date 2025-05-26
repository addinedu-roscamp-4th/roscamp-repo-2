import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

# 설정
correct_circles = 9
circle_cnt = 0

path = "/home/addinedu/Desktop/roscamp-repo-2/cook_gpt/cookgpt_service/substanceDiscrim/Hough/content/real-images/1-0.jpg"
image = cv2.imread(path)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
ret, filtered = cv2.threshold(gray, 120, 255, cv2.THRESH_TOZERO)
blurred = cv2.GaussianBlur(filtered, (9, 9), 5)

circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 70, param1=10, param2=25, minRadius=16, maxRadius=30)

if circles is not None:
    circles = np.uint16(np.around(circles))[0]

    # DBSCAN 클러스터링
    epsilon, minPts = 165, 2
    dbscan = DBSCAN(eps=epsilon, min_samples=minPts)
    dbscan.fit(circles[:, :2])
    labels = dbscan.labels_

    for i, (x, y, r) in enumerate(circles):
        cluster_id = labels[i]

        if cluster_id == -1: # 잘못 Detection된 원
            cv2.circle(image, (x, y), r, (0, 0, 255), 1)
            cv2.circle(image, (x, y), 2, (0, 0, 0), 3)
        else: # 클러스터된 제대로 Detection된 원
            cv2.circle(image, (x, y), r, (255, 0, 0), 1)
            cv2.circle(image, (x, y), 2, (0, 0, 0), 3)
            circle_cnt += 1

    clustered_circles = circles[labels != -1]

    # ROI 계산
    xs, ys, rs = clustered_circles[:, 0], clustered_circles[:, 1], clustered_circles[:, 2]
    rs_max = np.max(rs)
    h, w = image.shape[:2]

    x_min = int(max(np.min(xs - rs_max), 0))
    x_max = int(min(np.max(xs + rs_max), w - 1))
    y_min = int(max(np.min(ys - rs_max), 0))
    y_max = int(min(np.max(ys + rs_max), h - 1))

    cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)

    # 결과 출력
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if circle_cnt == correct_circles:
        print(f"[감지된 스티커 수: {circle_cnt} → 이상 없음 ✅")
    else:
        print(f"[감지된 스티커 수: {circle_cnt} → 이물질 감지 ⚠️")
else:
    print(f"스티커가 감지되지 않았습니다")