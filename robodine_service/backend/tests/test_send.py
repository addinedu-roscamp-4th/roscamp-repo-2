# robot_send.py
import socket
import json
import base64
import os

HOST = "127.0.0.1"    # 서버 IP
PORT = 8001           # TCP 포트

def send_chatbot_with_image(msg_id: int, robot_id: int, question: str,
                            text: str, img_path: str):
    # 1) 이미지 읽어서 Base64 인코딩
    with open(img_path, "rb") as f:
        img_bytes = f.read()
    b64img = base64.b64encode(img_bytes).decode("utf-8")

    # 2) 복합 페이로드 구성
    payload = {
        "msg_type":     "Chatbot",
        "msg_id":       msg_id,
        "question":     question,
        "robot_id":     robot_id,
        "robot_task":   "TAKE_PICTURE",
        "response_text": text,
        "response_image": b64img
    }

    raw = json.dumps(payload).encode("utf-8")
    # 3) 길이 헤더(4바이트 big-endian) + 데이터
    header = len(raw).to_bytes(4, byteorder="big")

    # 4) 전송
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((HOST, PORT))
        sock.sendall(header + raw)
        resp = sock.recv(1024)
        print("서버 응답:", resp.decode())

if __name__ == "__main__":
    # 예시: msg_id=1, robot_id=42, 질문·텍스트·이미지 파일 지정
    send_chatbot_with_image(
        msg_id=2,
        robot_id=42,
        question="사진 보여줘",
        text="여기 로봇이 찍은 사진입니다!",
        img_path="test.jpg"
    )
