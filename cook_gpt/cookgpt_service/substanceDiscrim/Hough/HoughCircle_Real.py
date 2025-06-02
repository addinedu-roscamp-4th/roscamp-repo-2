# cv2.HoughCircles의 파라미터 튜닝을 이용한 연구

import cv2
import numpy as np

correct_circles = 9
circle_cnt = 0

path = "./content/real-images/bad/1-0.jpg"
image = cv2.imread(path)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
ret, filtered = cv2.threshold(gray, 120, 255, cv2.THRESH_TOZERO)
blurred = cv2.GaussianBlur(filtered, (9, 9), 3)

# Hough Circles는 자체적으로 Canny Edge Detection을 수행
circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 70, param1=10, param2=25, minRadius=16, maxRadius=30)
circles = np.uint16(np.around(circles))

# x, y, r 각각 추출
xs = circles[0][:, 0]
ys = circles[0][:, 1]
rs = circles[0][:, 2]

rs_max = np.max(rs)
h, w = image.shape[:2]

# 최소/최대 좌표 계산 + 반지름 padding + 오차
x_min = np.min(xs - rs_max)
x_max = np.max(xs + rs_max)
y_min = np.min(ys - rs_max)
y_max = np.max(ys + rs_max)

# 정수형으로 변환
x_min = int(max(x_min, 0))
x_max = int(min(x_max,w-1))
y_min = int(max(y_min, 0))
y_max = int(min(y_max,h-1))

roi = image[y_min:y_max, x_min:x_max]
pts = np.array([[x_min, y_min], [x_min, y_max], [x_max, y_min], [x_max, y_max]], dtype=np.int32)

for circle in circles[0,:]:        
    if (x_min <= circle[0] and circle[0] <= x_max) and (y_min <= circle[1] and circle[1] <= y_max) : # ROI 영역 내에서 검출된 원들
        cv2.circle(image,(circle[0],circle[1]),circle[2],(255,0,0),1)
        cv2.circle(image,(circle[0],circle[1]),2,(0,0,0),3)
        circle_cnt = circle_cnt + 1
    else : # ROI 영역 바깥으로 검출된 원들
        cv2.circle(image,(circle[0],circle[1]),circle[2],(0,0,255),1)
        cv2.circle(image,(circle[0],circle[1]),2,(0,0,0),3)

cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)

if circle_cnt == 9 :
    print(f"감지된 스티커 수 : {circle_cnt}, 이상 없습니다")
else :
    print(f"감지된 스티커 수 : {circle_cnt}, 이물질이 감지되었습니다!!")