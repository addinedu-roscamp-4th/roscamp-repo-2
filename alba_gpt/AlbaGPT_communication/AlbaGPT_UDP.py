import socket
import numpy as np
import cv2
import json

def alba_udp_server(udp_stop_event, shared_data):
    """
    AlbaBot으로부터 온 영상 데이터를 받기 위해 UDP 서버를 열어주는 함수입니다.
    수신한 영상 데이터를 cv2로 띄워줍니다.
    """

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5000
    BUFFER_SIZE = 65535  # 일반적인 UDP 최대 수신 버퍼 크기

    # 소켓 설정
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT)) # 소켓 생성

    print(f"Listening on {UDP_IP}:{UDP_PORT}")

    while cv2.waitKey(1) != 27:
        try :
            packet, addr = sock.recvfrom(BUFFER_SIZE) # 소켓으로 들어온 값을 튜플로 받아온다
            transmitted_dict = json.loads(packet) # 패킷을 JSON 형태로 디코딩

            alba_video_byte = transmitted_dict['video_data'].encode('latin1')
            alba_id = transmitted_dict['alba_id']

            # 소켓에서 받은 패킷 디코딩
            np_vid_data = np.frombuffer(alba_video_byte, dtype=np.uint8)
            decoded_frame = cv2.imdecode(np_vid_data, cv2.IMREAD_COLOR)

            if decoded_frame is not None:
                shared_data.alba_id = alba_id
                shared_data.latest_frame = np_vid_data.tobytes() # 가장 최신 프레임 저장

        except KeyboardInterrupt :
            print("\nKeyboard Interrupt : Closing UDP Server and destroy all windows...")
            cv2.destroyAllWindows()
            udp_stop_event.set() # 스레드 종료 이벤트 설정
            break

    sock.close()
    
if __name__=="__main__":
    alba_udp_server(1, 1) # for debugging