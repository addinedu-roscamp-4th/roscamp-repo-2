# 이미 만들어진 영상으로 테스트 하는 코드

import cv2
import matplotlib.pyplot as plt
import numpy as np
import random
import time
from ultralytics import YOLO

# ---------------------------
# 1. 모델 로딩 및 영상/출력 설정
# ---------------------------
# 학습된 모델 로딩 (모델 경로는 상황에 맞게 수정)
model = YOLO('/home/addinedu/Desktop/Yolo_ws/yolov8n/runs/pose/train/weights/best.pt')

# 영상 파일 경로 (처리할 비디오 파일 경로 지정)
#video_path = "/home/addinedu/cobot_ws/video/01_steak_approach.mp4"
#video_path = "/home/addinedu/cobot_ws/video/06_katsu_fixed.mp4"
#video_path = "/home/addinedu/cobot_ws/video/mixed_04.mp4"
video_path = "/home/addinedu/cobot_ws/video/mixed_approach_01.mp4"

# 비디오 캡쳐 열기
cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print("비디오 파일을 열 수 없습니다:", video_path)
    exit()

# 비디오 속성 (FPS, 해상도 등)
fps = cap.get(cv2.CAP_PROP_FPS)
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# 처리 결과를 저장할 비디오 설정 (출력 파일 경로 및 코덱 지정)
output_path = "test_vid/output_pose_detection.mp4"
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

# keypoint 연결 관계 (예시: 총 27포인트의 경우 인접한 번호끼리 연결 — 필요에 따라 수정)
connections = [(i, i + 1) for i in range(26)]

# 무작위 색상을 반환하는 함수 (바운딩 박스 및 스켈레톤 선용)
def random_color():
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

# 단일 창으로 영상을 보여주기 위한 창 생성 (크기 조정 가능)
cv2.namedWindow("Pose Detection", cv2.WINDOW_NORMAL)

# ---------------------------
# 2. 영상 프레임 처리 루프 (모델 예측, 그리기, 좌표 출력)
# ---------------------------
frame_count = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break  # 영상의 모든 프레임을 처리하면 종료

    # 모델에 프레임 입력 (이미지 배열)
    results = model(frame)
    result = results[0]  # 단일 프레임 처리 결과

    # 터미널에 프레임 번호 출력
    print(f"\n=== Frame {frame_count} ===")
    
    # ---------------------------
    # 2-1. Pose Keypoints 및 스켈레톤 그리기 + 좌표 출력
    # ---------------------------
    if result.keypoints is not None:
        # 객체별 keypoints (형태: (num_objects, num_keypoints, 2))
        keypoints_all = result.keypoints.xy.cpu().numpy()
        
        # 각 객체에 대하여 좌표 출력 및 그리기
        for obj_idx, keypoints in enumerate(keypoints_all):
            print(f" Object {obj_idx} keypoints:")
            color = random_color()
            for kp_idx, (x, y) in enumerate(keypoints):
                if x > 0 and y > 0:
                    # 좌표 출력
                    print(f"   - Keypoint {kp_idx}: (x: {x:.1f}, y: {y:.1f})")
                    # 원 및 라벨 그리기
                    cv2.circle(frame, (int(x), int(y)), 5, color, -1)
                    cv2.putText(frame, str(kp_idx), (int(x), int(y)-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            # 스켈레톤 선 그리기
            for i, j in connections:
                if i < len(keypoints) and j < len(keypoints):
                    if all(keypoints[[i, j]].flatten() > 0):  # 두 keypoint 모두 유효한 경우
                        pt1 = (int(keypoints[i][0]), int(keypoints[i][1]))
                        pt2 = (int(keypoints[j][0]), int(keypoints[j][1]))
                        cv2.line(frame, pt1, pt2, color, 2)
    
    # ---------------------------
    # 2-2. 바운딩 박스 및 클래스 정보 그리기 + 좌표 출력
    # ---------------------------
    for box_idx, box in enumerate(result.boxes):
        xyxy = box.xyxy.cpu().numpy()      # 바운딩 박스 좌표 (형태: (1,4))
        cls  = box.cls.cpu().numpy()        # 클래스 id (예: [0])
        conf = box.conf.cpu().numpy()        # 신뢰도 (예: [0.85])
        class_name = result.names[int(cls)]
        conf_value = conf[0].item() if conf.size > 0 else 0.0
        
        # 좌표 정수형 변환
        x1, y1, x2, y2 = map(int, xyxy[0])
        print(f" Object {box_idx} bounding box: Class {class_name} (ID: {cls[0]}), Conf {conf_value:.2f}")
        print(f"   - Box coordinates: x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}")
        
        if (cls[0] == 0) : # Steak
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{class_name} {conf_value:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else : # donggs
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame, f"{class_name} {conf_value:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    # ---------------------------
    # 2-3. 처리된 프레임 저장 및 한 창에서 영상으로 표시
    # ---------------------------
    out.write(frame)  # 출력 비디오에 저장
    cv2.imshow("Pose Detection", frame)  # 하나의 창에서 연속적으로 보여줌

    # 'q' 키 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    frame_count += 1

end_time = time.time()
processing_time = end_time - start_time
print(f"\n총 프레임 수: {frame_count}, 전체 처리 시간: {processing_time:.2f}초")

# ---------------------------
# 3. 자원 해제
# ---------------------------
cap.release()
out.release()
cv2.destroyAllWindows()

# ---------------------------
# 4. 결과 영상 확인 (선택 사항: 저장된 영상 파일 첫 프레임을 matplotlib으로 확인)
# ---------------------------
cap_result = cv2.VideoCapture(output_path)
ret, sample_frame = cap_result.read()
if ret:
    sample_frame_rgb = cv2.cvtColor(sample_frame, cv2.COLOR_BGR2RGB)
    plt.figure(figsize=(10, 10))
    plt.imshow(sample_frame_rgb)
    plt.axis("off")
    plt.title("Processed Frame Sample")
    plt.show()
cap_result.release()