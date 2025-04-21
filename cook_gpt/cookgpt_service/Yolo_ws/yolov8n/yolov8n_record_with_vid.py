import cv2
import numpy as np
import requests
import threading
import time
import random
from ultralytics import YOLO
import os

# =============================================================================
# VideoStreamClient: 카메라 스트리밍 클래스 (MJPEG 스트림 수신)
# =============================================================================
class VideoStreamClient:
    def __init__(self, server_url):
        """
        비디오 스트림 클라이언트 초기화
        
        Args:
            server_url (str): 서버 스트림 URL (예: 'http://192.168.5.1:5000/stream')
        """
        self.server_url = server_url
        self.stream_active = False
        self.stream_thread = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # 녹화 관련 변수 (메인 루프에서 annotation된 프레임을 녹화할 예정)
        self.recording = False
        self.video_writer = None
        self.video_filename = None
        
    def start_stream(self):
        """스트림 수신 시작"""
        if self.stream_active:
            print("이미 스트림이 활성화되어 있습니다.")
            return
        self.stream_active = True
        self.stream_thread = threading.Thread(target=self._receive_stream)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
    def stop_stream(self):
        """스트림 수신 중지"""
        self.stream_active = False
        if self.stream_thread:
            self.stream_thread.join(timeout=1.0)
        self.stop_recording()
            
    def get_latest_frame(self):
        """최신 수신된 프레임 반환"""
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()
            
    def _receive_stream(self):
        """비디오 스트림 수신 및 처리하는 내부 메서드"""
        try:
            response = requests.get(self.server_url, stream=True)
            if response.status_code != 200:
                print(f"서버 연결 실패: {response.status_code}")
                self.stream_active = False
                return
                
            bytes_data = bytes()
            for chunk in response.iter_content(chunk_size=1024):
                bytes_data += chunk
                a = bytes_data.find(b'\xff\xd8')
                b = bytes_data.find(b'\xff\xd9')
                
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]
                    bytes_data = bytes_data[b+2:]
                    
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self.frame_lock:
                            self.latest_frame = frame
                if not self.stream_active:
                    break
                    
        except Exception as e:
            print(f"스트림 수신 중 오류 발생: {e}")
        finally:
            self.stream_active = False

    def start_recording(self, frame):
        """영상 녹화 시작 (메인 루프에서 처리된 frame의 크기를 기준으로 함)"""
        if self.recording:
            print("이미 녹화 중입니다.")
            return

        fourcc = cv2.VideoWriter_fourcc(*'mp4v') # 동영상 파일의 코덱을 지정
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.video_filename = os.path.join('./vid', f"recording_{timestamp}.mp4")

        height, width, _ = frame.shape
        self.video_writer = cv2.VideoWriter(self.video_filename, fourcc, 60.0, (width, height))
        self.recording = True
        print(f"녹화 시작: {self.video_filename}")
        
    def stop_recording(self):
        """영상 녹화 중지"""
        if self.recording:
            self.recording = False
            self.video_writer.release()
            self.video_writer = None
            print(f"녹화 완료: {self.video_filename}")
            self.video_filename = None

# =============================================================================
# 보조 함수: 무작위 색상 생성 및 keypoint 연결 관계 설정
# =============================================================================
def random_color():
    """바운딩 박스와 스켈레톤에 사용할 무작위 색상 반환"""
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) #(B, G, R) 값의 반환

# 예시로 27개의 keypoints가 있다고 가정하고, 인접한 점끼리 연결 (0~26)
connections = [(i, i + 1) for i in range(26)]

# =============================================================================
# 메인: 실시간 스트리밍 영상에 모델 적용 (몇 프레임마다 keypoints 업데이트 + 녹화)
# =============================================================================
if __name__ == "__main__":
    # 스트림 서버 URL (로봇팔 카메라 스트리밍 주소)
    server_url = "http://192.168.5.1:5000/stream"
    
    # YOLO Pose 모델 로딩 (모델 경로는 상황에 맞게 수정)
    model = YOLO('/home/addinedu/Desktop/Yolo_ws/yolov8n/runs/pose/train/weights/best.pt')
    
    # 스트리밍 클라이언트 생성 및 스트림 시작
    client = VideoStreamClient(server_url)
    client.start_stream()
    
    # 결과를 출력할 단일 창 생성 (크기 조절 가능)
    cv2.namedWindow("Pose Detection Stream", cv2.WINDOW_NORMAL)
    
    frame_count = 0
    start_time = time.time()
    
    # keypoints 그리기에 사용할 smoothed_keypoints_result 변수 (업데이트 간격마다 새로 갱신)
    smoothed_keypoints_result = None
    smoothing_interval = 5  # 예: 5프레임마다 keypoints 업데이트
    
    try:
        while True:
            frame = client.get_latest_frame()
            if frame is None:
                continue  # 프레임 없으면 대기
            
            # 모델 추론 (바운딩 박스는 실시간 업데이트)
            results = model(frame)
            result = results[0]  # 단일 프레임 결과
            
            print(f"\n=== Frame {frame_count} ===")
            
            # -----------------------------------------------------------------------------
            # 1. 매 smoothing_interval 프레임마다 keypoints 결과 업데이트
            # -----------------------------------------------------------------------------
            if result.keypoints is not None:
                if frame_count % smoothing_interval == 0:
                    # 새로운 keypoints 결과 저장 (여러 객체 처리 가능)
                    smoothed_keypoints_result = result.keypoints.xy.cpu().numpy()
                    # 터미널에 새 keypoints 결과 출력
                    for obj_idx, keypoints in enumerate(smoothed_keypoints_result):
                        print(f" Object {obj_idx} keypoints (업데이트):")
                        for kp_idx, (x, y) in enumerate(keypoints):
                            if x > 0 and y > 0:
                                print(f"   - Keypoint {kp_idx}: (x: {x:.1f}, y: {y:.1f})")
            
            # keypoints와 스켈레톤은 이전 업데이트 결과(smoothed_keypoints_result) 사용
            if smoothed_keypoints_result is not None:
                for obj_idx, keypoints in enumerate(smoothed_keypoints_result):
                    color = random_color()
                    for kp_idx, (x, y) in enumerate(keypoints):
                        if x > 0 and y > 0:
                            cv2.circle(frame, (int(x), int(y)), 5, color, -1)
                            cv2.putText(frame, str(kp_idx), (int(x), int(y)-5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                    # keypoints 간 연결 (스켈레톤)
                    for i, j in connections:
                        if i < len(keypoints) and j < len(keypoints):
                            if all(keypoints[[i, j]].flatten() > 0):
                                pt1 = (int(keypoints[i][0]), int(keypoints[i][1]))
                                pt2 = (int(keypoints[j][0]), int(keypoints[j][1]))
                                cv2.line(frame, pt1, pt2, color, 2)
            
            # -----------------------------------------------------------------------------
            # 2. 바운딩 박스 및 클래스 정보 그리기 + 터미널 출력
            # -----------------------------------------------------------------------------
            for box_idx, box in enumerate(result.boxes):
                xyxy = box.xyxy.cpu().numpy()      # 바운딩 박스 좌표 (형태: (1, 4))
                cls  = box.cls.cpu().numpy()        # 클래스 ID
                conf = box.conf.cpu().numpy()        # confidence 값
                class_name = result.names[int(cls)]
                conf_value = conf[0].item() if conf.size > 0 else 0.0
                
                x1, y1, x2, y2 = map(int, xyxy[0])
                print(f" Object {box_idx} bounding box: Class {class_name} (ID: {cls[0]}), Conf {conf_value:.2f}")
                print(f"   - Coordinates: x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}")
                
                if (cls[0] == 0) : # Steak
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{class_name} {conf_value:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else : # dongas
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, f"{class_name} {conf_value:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # -----------------------------------------------------------------------------
            # 3. 처리된(annotated) 프레임 녹화 및 스트리밍 창에 표시
            # -----------------------------------------------------------------------------
            # 녹화 기능: client.recording이 True이면, annotation이 완료된 frame을 녹화함.
            if client.recording and client.video_writer is not None:
                client.video_writer.write(frame)
                
            cv2.imshow("Pose Detection Stream", frame)
            
            frame_count += 1
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                if client.recording:
                    client.stop_recording()
                else:
                    client.start_recording(frame)
                        
    except KeyboardInterrupt:
        print("종료 중...")
    finally:
        elapsed = time.time() - start_time
        print(f"\n총 프레임 수: {frame_count}, 전체 처리 시간: {elapsed:.2f}초")
        client.stop_stream()
        cv2.destroyAllWindows()