import socket
import numpy as np
import cv2

def alba_udp_server(stop_event, shared_data):
    """
    AlbaBot으로부터 온 영상 데이터를 받기 위해 UDP 서버를 열어주는 함수입니다.
    """

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5000
    BUFFER_SIZE = 65535  # 일반적인 UDP 최대 수신 버퍼 크기

    global latest_frame
    
    frame = None

    # 소켓 설정
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening on {UDP_IP}:{UDP_PORT}")

    while not stop_event.is_set():
        try :
            packet, addr = sock.recvfrom(BUFFER_SIZE)
            
            # 소켓에서 받은 패킷 디코딩
            encoded_frame = np.frombuffer(packet, dtype=np.uint8)
            frame = cv2.imdecode(encoded_frame, cv2.IMREAD_COLOR)

            if frame is not None:
                shared_data.latest_frame = encoded_frame.tobytes() # 가장 최신 프레임 저장
                cv2.imshow("UDP Video Stream", frame)

            if cv2.waitKey(1) == 27:  # ESC 눌렀을 때 종료
                print("ESC pressed: Exiting...")
                stop_event.set()

        except KeyboardInterrupt :
            print("Keyboard Interrupt : close picam and terminate UDP Server...")
            stop_event.set() # 스레드 종료 이벤트 설정
            break

    sock.close()
    cv2.destroyAllWindows()
    return

# def get_latest_frame():
#     """
#     지정된 AlbaBot으로 부터 가장 최신 프레임을 받아오는 함수입니다.
#     """
    
#     if latest_frame is not None:
#         final_frame = latest_frame.copy()
#         recieved_image_path = 'test.jpg'

#         cv2.imwrite(recieved_image_path, final_frame)
#         print(f"Image successfully saved to {recieved_image_path}")
#         cv2.destroyAllWindows()

#         return latest_frame.copy()
#     else :
#         return