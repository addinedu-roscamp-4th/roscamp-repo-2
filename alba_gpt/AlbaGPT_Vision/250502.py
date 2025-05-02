import cv2
import os
import sys
import mediapipe as mp
import numpy as np
import time

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from multiprocessing import Process, Event, Manager

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from AlbaGPT_communication import AlbaGPT_UDP

def decoder_thread(decoder_stop_event, shared_frame):

    while not decoder_stop_event.is_set() :
        if shared_frame.latest_frame is None:
            continue
        
        else :
            np_data = np.frombuffer(shared_frame.latest_frame, dtype=np.uint8)
            shared_frame.decoded_image = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
            
    return

def mediapipe_thread(mp_stop_event, shared_frame) :
    # 미디어파이프 디텍터 모델 로드
    base_options = python.BaseOptions(model_asset_path='./model/efficientdet_lite0.tflite')
    VisionRunningMode = mp.tasks.vision.RunningMode
    options = vision.ObjectDetectorOptions(base_options=base_options, score_threshold=0.3, max_results=-1, # 전부 반환
                                           running_mode=VisionRunningMode.IMAGE, category_allowlist=["person", "mouse"])
    detector = vision.ObjectDetector.create_from_options(options)

    start_time = time.time()
    cnt = 0
    fps = 0

    # 디텍터 모델 사용
    with detector :
        while not mp_stop_event.is_set():
            decoded_image = shared_frame.decoded_image

            if decoded_image is None:
                continue

            cnt += 1

            current_time = time.time()
            elapsed_time = current_time - start_time

            if elapsed_time >= 1 :
                fps = cnt
                cnt = 0
                start_time = current_time

            # np.array -> Mediapipe Image
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=decoded_image)

            # 디텍션 수행
            results = detector.detect(mp_image)

            cv2.namedWindow('MediaPipe')
            
            if fps < 20 : 
                cv2.putText(decoded_image, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            elif fps >= 20 and fps < 30 :
                cv2.putText(decoded_image, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else :
                cv2.putText(decoded_image, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


            # 바운딩 박스를 원본 이미지에 그림
            for detection in results.detections:
                bbox = detection.bounding_box
                start_point = (int(bbox.origin_x), int(bbox.origin_y))
                end_point = (int(bbox.origin_x + bbox.width), int(bbox.origin_y + bbox.height))

                object = detection.categories[0].category_name
                x = int(detection.bounding_box.origin_x)
                y = int(detection.bounding_box.origin_y)
                width = detection.bounding_box.width
                height = detection.bounding_box.height

                if (object == "person") :
                    cv2.rectangle(decoded_image, start_point, end_point, (0, 255, 0), 2)
                    cv2.putText(decoded_image, f"{object}", (x, y + height + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                elif (object == "mouse") :
                    cv2.rectangle(decoded_image, start_point, end_point, (255, 0, 0), 2)
                    cv2.putText(decoded_image, f"{object}", (x, y + height + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # 결과 출력
            cv2.imshow(('MediaPipe'), decoded_image)

            if cv2.waitKey(1) == 27:  # ESC 눌렀을 때 종료
                print("Closing MediaPipe Thread...")
                cv2.destroyWindow("MediaPipe")
                mp_stop_event.set()
    return

if __name__=="__main__":
    udp_stop_event = Event()
    mp_stop_event = Event()
    decoder_stop_event = Event()

    manager = Manager()
    
    shared_frame = manager.Namespace()
    shared_frame.latest_frame = None
    shared_frame.decoded_image = None

    udp_thread = Process(target=AlbaGPT_UDP.alba_udp_server, args=(udp_stop_event, shared_frame))
    decoder_thread = Process(target=decoder_thread, args=(decoder_stop_event, shared_frame))
    mediapipe_thread = Process(target=mediapipe_thread, args=(mp_stop_event, shared_frame))

    udp_thread.start()
    decoder_thread.start()
    mediapipe_thread.start()

    udp_thread.join()
    decoder_thread.join()
    mediapipe_thread.join()