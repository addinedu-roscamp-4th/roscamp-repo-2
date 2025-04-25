import socket
import numpy as np
import cv2

UDP_IP = "0.0.0.0"
UDP_PORT = 5000
BUFFER_SIZE = 65535  # 일반적인 UDP 최대 수신 버퍼 크기

# 소켓 설정
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on {UDP_IP}:{UDP_PORT}")

while True:
    packet, addr = sock.recvfrom(BUFFER_SIZE)
    
    # 디코딩
    np_data = np.frombuffer(packet, dtype=np.uint8)
    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

    if frame is not None:
        cv2.imshow("UDP Video Stream", frame)

    if cv2.waitKey(1) == 27:  # ESC 키로 종료
        break

sock.close()
cv2.destroyAllWindows()
