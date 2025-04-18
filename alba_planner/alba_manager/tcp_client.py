#alba_manager/tcp_client.py

import socket
import json
from datetime import datetime

HOST = '192.168.0.156'  # 메인 서버 IP 주소
PORT = 8001

data = {
    "id": "string4",  # 필수
    "battery_level": 90,  # 변경하고 싶은 값만 포함
    "timestamp": "2025-04-18T03:23:12.765Z"
}

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    try:
        s.connect((HOST, PORT))
        s.sendall(json.dumps(data).encode('utf-8'))
        response = s.recv(1024)
        print(f"서버 응답: {response.decode('utf-8')}")
    except Exception as e:
        print(f"연결 또는 전송 실패: {e}")

        
