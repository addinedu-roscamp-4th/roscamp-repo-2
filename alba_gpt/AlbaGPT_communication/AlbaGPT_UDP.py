import socket
import numpy as np
import cv2
import time

def alba_udp_server(udp_stop_event, shared_frame):
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

    while True:
        try :
            packet, addr = sock.recvfrom(BUFFER_SIZE)
            
            # 소켓에서 받은 패킷 디코딩
            encoded_frame = np.frombuffer(packet, dtype=np.uint8)
            frame = cv2.imdecode(encoded_frame, cv2.IMREAD_COLOR)

            if frame is not None:
                shared_frame.latest_frame = encoded_frame.tobytes() # 가장 최신 프레임 저장

        except KeyboardInterrupt :
            print("\nKeyboard Interrupt : close picam and terminate UDP Server...")
            udp_stop_event.set() # 스레드 종료 이벤트 설정
            break

    sock.close()
    return