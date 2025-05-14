import cv2
import mediapipe as mp
import numpy as np
import time
import AlbaGPT_function
import socket
import json
import logging
import requests

from AlbaGPT_communication import transfer_payloads
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from multiprocessing import Process, Event, Manager

alba_task_type_list = ["GREETINGS", "TAKE_PICTURE", "MAINTENANCE"]
dynamic_object_list = ["person", "bird", "cat", "dog"]

# 로거 생성
logger = logging.getLogger('alba_main')
logger.setLevel(logging.INFO)

if not logger.hasHandlers():
    # 핸들러 생성 (파일에 기록)
    file_handler = logging.FileHandler('./contents/log/alba_main.log')
    formatter = logging.Formatter('[%(asctime)s] %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

memory_url = 'http://192.168.0.156:8000/api/chat/history?page=1&per_page=3' # AlbaBot의 응답을 저장해주는 url

if not logger.handlers:
    # 핸들러 생성 (파일에 기록)
    file_handler = logging.FileHandler('alba_main.log')
    formatter = logging.Formatter('[%(asctime)s] %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)

    # 로거에 핸들러 추가
    logger.addHandler(file_handler)

def alba_draw_bbox(decoded_image, results, fps, dynamic_object_list=dynamic_object_list) :
    """
    MediaPipe를 통해 검출된 객체의 바운딩 박스를 그려주는 함수입니다.

    Returns :
        None
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

        if object in dynamic_object_list :
            cv2.rectangle(decoded_image, start_point, end_point, (0, 0, 255), 2)
            cv2.putText(decoded_image, f"{object}", (x, y + height + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)    
            
    if fps < 20 : 
        cv2.putText(decoded_image, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    elif fps >= 20 and fps < 60 :
        cv2.putText(decoded_image, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else :
        cv2.putText(decoded_image, f"fps : {fps}", (550, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)    


def alba_udp_thread(udp_stop_event, shared_dict, manager) :
    """
    UDP 서버를 열어주는 함수입니다.
    AlbaBot으로부터 실시간으로 영상 데이터를 받아 디코딩하고, Mediapipe로 동적 장애물을 검출합니다.

    Returns :
        None
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

    logger.info(f"🌐 UDP Listening on {UDP_IP}:{UDP_PORT}")

    start_time = time.time()
    cnt = 0
    fps = 0

    with detector :
        while cv2.waitKey(1) != 27:
            try :
                packet, addr = sock.recvfrom(BUFFER_SIZE) # 소켓으로 들어온 값을 튜플로 받아온다
                transmitted_dict = json.loads(packet) # 패킷을 JSON 형태로 디코딩

                alba_video_byte = transmitted_dict["video_data"].encode('latin1')
                robot_id = transmitted_dict["robot_id"]
                ip_port = f"{addr[0]}:{addr[1]}"

                if ip_port not in shared_dict:
                    logger.info(f"🔌 [INFO] New UDP connection detected from {ip_port}")
                    shared_sub_dict = manager.dict()
                    shared_sub_dict["latest_frame"] = None
                    shared_sub_dict["detected_object"] = None
                    shared_dict[ip_port] = shared_sub_dict

                if robot_id not in shared_dict:
                    shared_id_dict = manager.dict()
                    shared_id_dict["ip_port"] = ip_port
                    shared_dict[robot_id] = shared_id_dict

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
                alba_draw_bbox(decoded_frame, results, fps)

                # 결과 출력
                cv2.imshow(f"AlbaBot {robot_id} ({ip_port}) streaming...", decoded_frame)

                # 결과 이미지를 JPEG 형식으로 인코딩
                success, encoded_bbox_image = cv2.imencode('.jpg', decoded_frame)

                if success :
                    shared_sub_dict["latest_frame"] = encoded_bbox_image.tobytes()
                    shared_sub_dict["detected_object"] = results.detections

            except KeyboardInterrupt :
                logging.warning(f"🛑 Keyboard Interrupt : Closing UDP Server and destroy all windows...")
                cv2.destroyAllWindows()
                udp_stop_event.set() # 스레드 종료 이벤트 설정
                sock.close()
                break

def alba_tcp_thread(tcp_stop_event, shared_dict):
    """
    TCP 서버를 열어주는 함수입니다.
    """

    TCP_IP = '0.0.0.0'
    TCP_PORT = 8001
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((TCP_IP, TCP_PORT))
    server_socket.listen(5)
    logger.info(f"🌐 TCP Listening on {TCP_IP}:{TCP_PORT}")

    try:
        client_socket, client_address = server_socket.accept()
        logger.info(f"🔌 New TCP connection from {client_address}")
        hdr = client_socket.recv(4)
        length = int.from_bytes(hdr, byteorder='big')
        payload_bytes = b''

        while len(payload_bytes) < length:
            chunk = client_socket.recv(length - len(payload_bytes))
            if not chunk:
                raise ConnectionError("Incomplete payload")
            payload_bytes += chunk

        payload = json.loads(payload_bytes.decode('utf-8'))
        msg_id = payload.get('msg_id')
        user_query = payload.get('question')

        logger.info(f"📦 Parsed msg_id: {msg_id}")
        logger.info(f"📦 Parsed user_query: {user_query}")

        memory = requests.get(memory_url).json()
        robot_task = AlbaGPT_function.validate_alba_task_discriminator(user_query, memory)
        robot_id = AlbaGPT_function.extract_robot_id(user_query)

        logger.info(f"🧠 Previous memory : {memory}")
        logger.info(f"💼 Detected Alba Task : {robot_task}")
        logger.info(f"🪪 Detected Alba ID : {robot_id}")

        if robot_task == "GREETINGS":
            response = AlbaGPT_function.generate_alba_greetings_response(user_query, memory)
            transfer_payloads(msg_id, user_query, robot_id, robot_task, response, None, tcp_stop_event)
        elif robot_task == "TAKE_PICTURE":
            response, img_path = AlbaGPT_function.generate_alba_take_picture_response(user_query, memory, shared_dict, robot_id)
            transfer_payloads(msg_id, user_query, robot_id, robot_task, response, img_path, tcp_stop_event)
        elif robot_task == "MAINTENANCE":
            response = AlbaGPT_function.generate_alba_maintenance_response(user_query, memory)
            transfer_payloads(msg_id, user_query, robot_id, robot_task, response, None, tcp_stop_event)
        else:
            response = AlbaGPT_function.generate_alba_none_response(user_query)
            transfer_payloads(msg_id, user_query, robot_id, robot_task, response, None, tcp_stop_event)

        client_socket.sendall(len(response.encode()).to_bytes(4, 'big') + response.encode())
        client_socket.close()
    except Exception as e:
        logger.warning(f"TCP Error: {e}")
    finally:
        server_socket.close()

if __name__=="__main__":
    tcp_stop_event = Event()
    udp_stop_event = Event()
    manager = Manager()

    shared_dict = manager.dict()

    udp_process = Process(target=alba_udp_thread, args=(udp_stop_event, shared_dict, manager))
    udp_process.start()
    
    while True:
        if tcp_stop_event.is_set() :
            tcp_stop_event.clear()

        tcp_process = Process(target=alba_tcp_thread, args=(tcp_stop_event, shared_dict))
        tcp_process.start()
        tcp_process.join()


        if not tcp_process.is_alive():
            logger.info("🔁 TCP thread completed successfully.")
        else:
            logger.warning("⚠️ TCP process still alive. Check socket logic.")
        time.sleep(1)  # CPU 과부하 방지