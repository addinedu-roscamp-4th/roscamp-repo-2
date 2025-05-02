import socket
import numpy as np
import cv2

def alba_udp_server(udp_stop_event, shared_frame):
    """
    AlbaBot으로부터 온 영상 데이터를 받기 위해 UDP 서버를 열어주는 함수입니다.
    수신한 영상 데이터를 cv2로 띄워줍니다.
    """

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5000
    BUFFER_SIZE = 65535  # 일반적인 UDP 최대 수신 버퍼 크기

    decoded_frame = None

    # 소켓 설정
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening on {UDP_IP}:{UDP_PORT}")

    while True:
        try :
            packet, addr = sock.recvfrom(BUFFER_SIZE)
            
            # 소켓에서 받은 패킷 디코딩
            np_data = np.frombuffer(packet, dtype=np.uint8)
            decoded_frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

            if decoded_frame is not None:
                shared_frame.latest_frame = np_data.tobytes() # 가장 최신 프레임 저장

        except KeyboardInterrupt :
            print("\nKeyboard Interrupt : Closing UDP Server and destroy all windows...")
            cv2.destroyAllWindows()
            udp_stop_event.set() # 스레드 종료 이벤트 설정
            break

    sock.close()