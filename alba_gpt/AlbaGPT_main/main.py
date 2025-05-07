import os
import sys
import cv2
import os
import sys
import mediapipe as mp
import numpy as np
import time
import AlbaGPT_function
import socket
import json

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from multiprocessing import Process, Event, Manager
from langchain.memory import ConversationBufferMemory

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from AlbaGPT_communication import AlbaGPT_UDP

alba_work_type_list = ["CLEANING", "SERVING", "MAINTENANCE", "EMERGENCY", "IDLE"]
alba_task_type_list = alba_work_type_list + ["GREETINGS", "CAMERA_ON"]
dynamic_object_list = ["person", "bird", "cat", "dog"]

def draw_boundingbox_fps(decoded_image, results, fps, dynamic_object_list=dynamic_object_list) :
    """
    MediaPipe의 객체 검출 결과를 토대로 바운딩 박스와, fps를 그려주는 함수입니다.
    """
    for detection in results.detections:
        bbox = detection.bounding_box
        start_point = (int(bbox.origin_x), int(bbox.origin_y))
        end_point = (int(bbox.origin_x + bbox.width), int(bbox.origin_y + bbox.height))

        object = detection.categories[0].category_name
        x = int(detection.bounding_box.origin_x)
        y = int(detection.bounding_box.origin_y)
        width = detection.bounding_box.width
        height = detection.bounding_box.height

        shared_data.dynamic_object_list = dynamic_object_list

        if object in dynamic_object_list :
            cv2.rectangle(decoded_image, start_point, end_point, (0, 0, 255), 2)
            cv2.putText(decoded_image, f"{object}", (x, y + height + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)    
        else :
            cv2.rectangle(decoded_image, start_point, end_point, (0, 255, 0), 2)
            cv2.putText(decoded_image, f"{object}", (x, y + height + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
        if fps < 20 : 
            cv2.putText(decoded_image, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        elif fps >= 20 and fps < 60 :
            cv2.putText(decoded_image, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else :
            cv2.putText(decoded_image, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)    

def alba_server_thread(mp_stop_event, shared_dict, shared_data, manager) :
    """
    AlbaBot으로부터 실시간으로 영상 데이터를 받아 디코딩하고, Mediapipe로 동적 / 정적 장애물 객체를 검출합니다.
    """
    # 미디어파이프 디텍터 모델 로드
    base_options = python.BaseOptions(model_asset_path='./model/efficientdet_lite0.tflite')
    VisionRunningMode = mp.tasks.vision.RunningMode
    options = vision.ObjectDetectorOptions(base_options=base_options, score_threshold=0.4, max_results=-1, # 전부 반환
                                           running_mode=VisionRunningMode.IMAGE)
    detector = vision.ObjectDetector.create_from_options(options)

    # 서버 설정
    UDP_IP = "0.0.0.0"
    UDP_PORT = 5000
    BUFFER_SIZE = 65535  # 일반적인 UDP 최대 수신 버퍼 크기

    # 소켓 설정
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT)) # 소켓 생성

    print(f"Listening on {UDP_IP}:{UDP_PORT}")

    start_time = time.time()
    cnt = 0
    fps = 0

    with detector :
        while cv2.waitKey(1) != 27:
            try :
                packet, addr = sock.recvfrom(BUFFER_SIZE) # 소켓으로 들어온 값을 튜플로 받아온다
                transmitted_dict = json.loads(packet) # 패킷을 JSON 형태로 디코딩

                alba_video_byte = transmitted_dict['video_data'].encode('latin1')
                alba_id = transmitted_dict['alba_id']
                ip_port = f"{addr[0]}:{addr[1]}"

                if ip_port not in shared_dict:
                    print(f"[INFO] New connection detected from {ip_port}")
                    shared_sub_dict = manager.dict()
                    shared_sub_dict["latest_frame"] = None
                    shared_sub_dict["alba_id"] = None
                    shared_sub_dict["address"] = None
                    shared_sub_dict["detected_object"] = None
                    shared_dict[ip_port] = shared_sub_dict

                # 소켓에서 받은 패킷 디코딩
                np_vid_data = np.frombuffer(alba_video_byte, dtype=np.uint8)
                decoded_frame = cv2.imdecode(np_vid_data, cv2.IMREAD_COLOR)
                decoded_frame = cv2.resize(decoded_frame, (640, 480))

                if decoded_frame is None:
                    continue

                cnt += 1

                current_time = time.time()
                elapsed = current_time - start_time
                
                if elapsed >= 1 :
                    fps = cnt
                    cnt = 0
                    start_time = current_time

                # np.array -> Mediapipe Image
                mediapipe_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=decoded_frame)

                # 디텍션 수행
                results = detector.detect(mediapipe_image)

                # 바운딩 박스, fps 그리기
                draw_boundingbox_fps(decoded_frame, results, fps)

                # 결과 출력
                cv2.imshow(f"AlbaBot {alba_id} ({ip_port}) streaming...", decoded_frame)

                # 결과 이미지를 JPEG 형식으로 인코딩
                success, encoded_bbox_image = cv2.imencode('.jpg', decoded_frame)

                if success :
                    shared_sub_dict["latest_frame"] = encoded_bbox_image.tobytes()
                    shared_sub_dict["alba_id"] = alba_id
                    shared_sub_dict["address"] = ip_port
                    shared_sub_dict["detected_object"] = results.detections

            except KeyboardInterrupt :
                print("\nKeyboard Interrupt : Closing UDP Server and destroy all windows...")
                cv2.destroyAllWindows()
                mp_stop_event.set() # 스레드 종료 이벤트 설정
                sock.close()
                break

def main_thread(shared_dict, shared_data) :
    memory = ConversationBufferMemory(memory_key="chat_history", input_key="user_query")
    chat_history = memory.load_memory_variables({})["chat_history"]

    user_query_example_list = ["1번 핑키 안녕!", "오늘 기분이 어때?, 1번 핑키?","3 테이블 청소해, 4번 핑키!",
                               "핑키들, 지금 매장에 도둑이 들었어!", "2번 핑키 4번 테이블이 더럽잖아.", "5번 테이블에 셀러드 하나, 2번 핑키!",
                               "핑키들 5번 테이블 손님 오늘 생일이래", "3번 핑키 베터리가 부족해 보여", "1번 핑키 카메라 켜봐", "2번 핑키 고생했어, 원상 복귀"]

    #user_query_example_list = ["3번 핑키 카메라 켜봐"]

    for user_query in user_query_example_list :
        discriminated_task = AlbaGPT_function.validate_alba_task_discriminator(user_query)
        print(discriminated_task)

        if discriminated_task == "GREETINGS" :
            alba_response = AlbaGPT_function.generate_alba_greetings_response(user_query, chat_history, memory)
            AlbaGPT_function.save_alba_response("GREETINGS", alba_response)

        elif discriminated_task == "CAMERA_ON" :
            alba_response = AlbaGPT_function.generate_alba_camera_on_response(chat_history, shared_dict, shared_data)
            AlbaGPT_function.save_alba_response("CAMERA_ON", alba_response)

        else :
            for work in alba_work_type_list :
                if discriminated_task == work :
                    alba_response = AlbaGPT_function.generate_alba_work_response(user_query, chat_history, memory, work)
                    AlbaGPT_function.save_alba_response(discriminated_task, alba_response)

if __name__=="__main__":
    # udp_stop_event = Event()
    mp_stop_event = Event()
    manager = Manager()

    shared_dict = manager.dict()
    shared_data = manager.Namespace()

    shared_data.dynamic_object_list = dynamic_object_list

    # udp_process = Process(target=AlbaGPT_UDP.alba_udp_server, args=(udp_stop_event,shared_data))
    vision_thread = Process(target=alba_server_thread, args=(mp_stop_event, shared_dict, shared_data, manager))
    main_process = Process(target=main_thread, args=(shared_dict, shared_data,))

    # udp_process.start()
    vision_thread.start()
    main_process.start()

    # udp_process.join()
    vision_thread.join()
    main_process.join()