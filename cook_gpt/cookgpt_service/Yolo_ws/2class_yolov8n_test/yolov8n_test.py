import cv2
import matplotlib.pyplot as plt
import numpy as np
from ultralytics import YOLO
import random
import time

# 모델 로딩
model = YOLO("runs/pose/train12/weights/best.pt")
# 이미지 경로
img_path = "addineddu/vision/source/StickerPoseDetection.v2i (2).yolov8/train/images/01_steak_approach_mp4-0106_jpg.rf.7671784a135e756d3b93b367efd0e94f.jpg"
# 예측 시간 측정 시작
start_time = time.time()
# 예측
results = model(img_path)
# 예측 시간 측정 종료
end_time = time.time()
# 이미지 로드 및 RGB로 변환
img = cv2.imread(img_path)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# 정의: keypoint 연결 관계 (예시: 인접한 점만 연결 — 원하는 대로 수정 가능)
connections = [(i, i + 1) for i in range(26)]  # 27점 기준 (0~26)
# 색상 팔레트
def random_color():
    return tuple([random.randint(0, 255) for _ in range(3)])
# 객체별로 keypoints 처리
for idx, result in enumerate(results):
    if result.keypoints is not None:
        kpts = result.keypoints.xy.cpu().numpy()
        for obj in kpts:
            color = random_color()
            # 점 그리기 + 라벨
            for i, (x, y) in enumerate(obj):
                if x > 0 and y > 0:
                    cv2.circle(img, (int(x), int(y)), 5, color, -1)
                    cv2.putText(img, str(i), (int(x), int(y) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            # 선 연결 (인덱스가 유효한 점만 연결)
            for i, j in connections:
                if i < len(obj) and j < len(obj):  # 유효한 점만 연결
                    if all(obj[[i, j]].flatten() > 0):  # 유효한 점만 연결
                        pt1 = tuple(map(int, obj[i]))
                        pt2 = tuple(map(int, obj[j]))
                        cv2.line(img, pt1, pt2, color, 2)
# 객체 감지 정보 출력
for idx, box in enumerate(results[0].boxes):  # results[0]로 첫 번째 이미지의 결과 가져오기
    xyxy = box.xyxy.cpu().numpy()  # 바운딩 박스 좌표
    cls = box.cls.cpu().numpy()    # 클래스 ID
    conf = box.conf.cpu().numpy()  # confidence
    class_name = results[0].names[int(cls)]  # 클래스 이름 (results[0]로 첫 번째 결과 접근)
    # 여기서 conf는 배열이므로, 첫 번째 값만 사용
    conf_value = conf[0].item()  # .item()으로 스칼라 값 추출
    print(f"Object {idx}: Class {class_name} (ID: {cls}), Confidence: {conf_value:.2f}, XYXY: {xyxy}")
    # 바운딩 박스 그리기 + 클래스 이름 및 confidence 표시
    cv2.rectangle(img,
                  (int(xyxy[0][0]), int(xyxy[0][1])),  # .item() 사용하지 않고 직접 추출
                  (int(xyxy[0][2]), int(xyxy[0][3])),
                  (0, 255, 0), 2)
    cv2.putText(img, f"{class_name} {conf_value:.2f}",
                (int(xyxy[0][0]), int(xyxy[0][1]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
# keypoints 정보 출력
for result in results:
    if result.keypoints is not None:
        keypoints = result.keypoints.xy.cpu().numpy()
        confidences = result.keypoints.conf.cpu().numpy()  # 신뢰도
        for i, (x, y) in enumerate(keypoints[0]):
            print(f"Keypoint {i}: (x: {x}, y: {y}), Confidence: {confidences[0][i]:.2f}")
# 예측 속도 출력
inference_time = end_time - start_time
print(f"Inference Time: {inference_time:.2f} seconds")
# 시각화
plt.figure(figsize=(12, 12))
plt.imshow(img)
plt.axis('off')
plt.title("Pose Keypoints + Skeletons with Confidence and Bounding Boxes")
plt.show()