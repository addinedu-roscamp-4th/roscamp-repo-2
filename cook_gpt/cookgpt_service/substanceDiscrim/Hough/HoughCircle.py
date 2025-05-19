#-*-coding:utf-8-*-

import cv2
import numpy as np

img = cv2.imread('content/correct/correct7.jpg',0)
# img = cv2.imread('content/error/error6.jpg',0)
cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

cnt = 0
correct = 5

circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 100, param1=20, param2=15, minRadius=5, maxRadius=15)
circles = np.uint16(np.around(circles))

for i in circles[0,:]:
    cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),1)
    cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),2)
    cnt = cnt + 1

if cnt != correct :
    print("[ERROR] 이물질이 있습니다!!")
else :
    print("[CORRECT] 아무 이상 없습니다!!")

while True :
    cimg = cv2.resize(cimg, (600, 600))
    cv2.imshow('test img', cimg)
    cv2.waitKey(0)
    if cv2.waitKey() == 27:
        cv2.destroyAllWindows()
        break