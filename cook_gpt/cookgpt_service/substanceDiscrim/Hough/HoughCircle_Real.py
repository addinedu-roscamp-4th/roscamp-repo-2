import cv2
import numpy as np

# 원본 이미지 로드
image = cv2.imread('./content/real-images/3-0.jpg')

pts = np.array([[181, 22], [493, 65], [418, 412], [83, 337]], dtype=np.int32) # ROI

# 빈 마스크 생성
mask = np.zeros_like(image)

# ROI를 흰색으로 채우기
cv2.fillPoly(mask, [pts], (255, 255, 255))

# 마스킹 영역만 남기기
cropped = cv2.bitwise_and(image, mask)
cropped_gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

# 이미지에서 Edge를 찾기
contours, hierachy = cv2.findContours(cropped_gray.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

rect = cv2.minAreaRect(contours[0])
rotated_angle = rect[2]

(h, w) = image.shape[:2]
center = (w // 2, h // 2)

# 회전 행렬 계산
M = cv2.getRotationMatrix2D(center, rotated_angle, 1.0)

# 경계가 잘리지 않도록 전체 이미지 크기로 회전 적용
rotated = cv2.warpAffine(cropped, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)

cnt = 0
correct = 5

circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 100, param1=20, param2=15, minRadius=5, maxRadius=15)
circles = np.uint16(np.around(circles))

for i in circles[0,:]:
    cv2.circle(rotated,(i[0],i[1]),i[2],(0,255,0),1)
    cv2.circle(rotated,(i[0],i[1]),2,(0,0,255),2)
    cnt = cnt + 1

if cnt != correct :
    print("[ERROR] 이물질이 있습니다!!")
else :
    print("[CORRECT] 아무 이상 없습니다!!")

while True :
    rotated = cv2.resize(rotated, (600, 600))
    cv2.imshow('Results', rotated)
    cv2.waitKey(0)
    if cv2.waitKey() == 27:
        cv2.destroyAllWindows()
        break