import os
import sys
import cv2
import os
import sys
import mediapipe as mp
import numpy as np
import time
import AlbaGPT_function

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from multiprocessing import Process, Event, Manager
from langchain.memory import ConversationBufferMemory

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from AlbaGPT_communication import AlbaGPT_UDP

alba_work_type_list = ["CLEANING", "SERVING", "MAINTENANCE", "EMERGENCY", "IDLE"]
alba_task_type_list = alba_work_type_list + ["GREETINGS", "CAMERA_ON"]

def mediapipe_thread(mp_stop_event, shared_data) :
    """
    동적 장애물을 검출하기 위한 함수입니다. AlbaBot으로부터 받은 이미지 프레임을 디코딩 하고, MediaPipe 모델을 통과시켜 검출한 객체를 바운딩 박스로 그려 cv2로 띄워주는 역할을 수행합니다.
    검출할 동적 장애물은 아래와 같습니다.

    ["person", "bird", "cat", "dog"]

    """

    # 미디어파이프 디텍터 모델 로드
    base_options = python.BaseOptions(model_asset_path='./model/efficientdet_lite0.tflite')
    VisionRunningMode = mp.tasks.vision.RunningMode
    options = vision.ObjectDetectorOptions(base_options=base_options, score_threshold=0.4, max_results=-1, # 전부 반환
                                           running_mode=VisionRunningMode.IMAGE)
    detector = vision.ObjectDetector.create_from_options(options)

    start_time = time.time()
    cnt = 0
    fps = 0

    # 디텍터 모델 사용
    with detector :
        while not mp_stop_event.is_set():
            if shared_data.latest_frame is None:
                continue

            np_data = np.frombuffer(shared_data.latest_frame, dtype=np.uint8)
            decoded_image = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
            
            cnt += 1

            current_time = time.time()
            elapsed = current_time - start_time
            

            if elapsed >= 1 :
                fps = cnt
                cnt = 0
                start_time = current_time

            # np.array -> Mediapipe Image
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=decoded_image)

            # 디텍션 수행
            results = detector.detect(mp_image)
            
            if fps < 20 : 
                cv2.putText(decoded_image, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            elif fps >= 20 and fps < 60 :
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

                dynamic_object_list = ["person", "bird", "cat", "dog"]
                shared_data.dynamic_object_list = dynamic_object_list

                if object in dynamic_object_list :
                    cv2.rectangle(decoded_image, start_point, end_point, (0, 0, 255), 2)
                    cv2.putText(decoded_image, f"{object}", (x, y + height + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)    
                else :
                    cv2.rectangle(decoded_image, start_point, end_point, (0, 255, 0), 2)
                    cv2.putText(decoded_image, f"{object}", (x, y + height + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)    


            # 결과 출력
            cv2.imshow('Alba Detection Streaming...', decoded_image)

            # 결과 이미지를 JPEG 형식으로 인코딩
            success, encoded_image = cv2.imencode('.jpg', decoded_image)
            if success :
                shared_data.detected_frame = encoded_image.tobytes()
                shared_data.detected_object = results.detections

            if cv2.waitKey(1) == 27:  # ESC 눌렀을 때 종료
                print("Closing Detection Result...")
                mp_stop_event.set()

def main_thread(shared_data) :
    memory = ConversationBufferMemory(memory_key="chat_history", input_key="user_query")
    chat_history = memory.load_memory_variables({})["chat_history"]

    # user_query_example_list = ["1번 핑키 안녕!", "오늘 기분이 어때?, 1번 핑키?","3 테이블 청소해, 4번 핑키!",
    #                            "핑키들, 지금 매장에 도둑이 들었어!", "2번 핑키 4번 테이블이 더럽잖아.", "5번 테이블에 셀러드 하나, 2번 핑키!",
    #                            "핑키들 5번 테이블 손님 오늘 생일이래", "3번 핑키 베터리가 부족해 보여", "2번 핑키 고생했어, 원상 복귀", "1번 핑키 카메라 켜봐"]

    user_query_example_list = ["2번 핑키 카메라 켜봐"]

    for user_query in user_query_example_list :
        discriminated_task = AlbaGPT_function.validate_alba_task_discriminator(user_query)
        print(discriminated_task)

        if discriminated_task == "GREETINGS" :
            alba_response = AlbaGPT_function.generate_alba_greetings_response(user_query, chat_history, memory)
            AlbaGPT_function.save_alba_response("GREETINGS", alba_response)

        elif discriminated_task == "CAMERA_ON" :
            alba_response = AlbaGPT_function.generate_alba_camera_on_response(chat_history, shared_data)
            AlbaGPT_function.save_alba_response("CAMERA_ON", alba_response)

        else :
            for work in alba_work_type_list :
                if discriminated_task == work :
                    alba_response = AlbaGPT_function.generate_alba_work_response(user_query, chat_history, memory, work)
                    AlbaGPT_function.save_alba_response(discriminated_task, alba_response)

if __name__=="__main__":
    udp_stop_event = Event()
    mp_stop_event = Event()
    manager = Manager()
    
    shared_data = manager.Namespace()
    shared_data.latest_frame = None
    shared_data.detected_frame = None
    shared_data.detected_object = None
    shared_data.dynamic_object_list = None

    udp_process = Process(target=AlbaGPT_UDP.alba_udp_server, args=(udp_stop_event,shared_data))
    mediapipe_thread = Process(target=mediapipe_thread, args=(mp_stop_event, shared_data))
    main_process = Process(target=main_thread, args=(shared_data,))

    udp_process.start()
    mediapipe_thread.start()
    main_process.start()

    udp_process.join()
    mediapipe_thread.join()
    main_process.join()