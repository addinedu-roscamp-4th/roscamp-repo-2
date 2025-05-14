import socket
import json
import base64
import os
import logging

# 로거 생성
logger = logging.getLogger('payload_transfer')
logger.setLevel(logging.INFO)

# 핸들러 생성 (파일에 기록)
file_handler = logging.FileHandler('./contents/log/payload_transfer.log')
formatter = logging.Formatter('[%(asctime)s] %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)

# 로거에 핸들러 추가
logger.addHandler(file_handler)

HOST = "192.168.0.156"    # 서버 IP
PORT = 8001               # TCP 포트

def transfer_payloads(msg_id: int, question: str, robot_id: int, robot_task: str, response_text: str, img_path: str, tcp_stop_event=None):
    base64_img = None

    # 1) 이미지 읽어서 Base64 인코딩
    if img_path is not None and os.path.exists(img_path):
        logger.info(f"📸 Image found at {img_path}, encoding to Base64...")
        with open(img_path, "rb") as f:
            img_bytes = f.read()
        base64_img = base64.b64encode(img_bytes).decode("utf-8")
        logger.info(f"✅ Image successfully encoded.")

    # 2) 복합 페이로드 구성
    payload = {
        "msg_type": "Chatbot",
        "msg_id": msg_id,
        "question": question,
        "robot_id": robot_id,
        "robot_task": robot_task,
        "response_text": response_text,
    }

    if base64_img is not None:
        payload["response_image"] = base64_img
    else :
        payload["response_image"] = ""

    raw = json.dumps(payload).encode("utf-8")
    header = len(raw).to_bytes(4, byteorder="big")

    logger.info("📤 Payload contents:\n%s", json.dumps(payload, indent=4, ensure_ascii=False))
    logger.info(f"🗂 Header prepared: {header.hex()} (length: {len(raw)} bytes)")

    # 3) 페이로드 전송
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((HOST, PORT))
            logger.info(f"🌐 Connecting to {HOST}:{PORT}...")

            sock.sendall(header + raw)
            logger.info(f"🚀 Payload sent to {HOST}:{PORT}")

            resp = sock.recv(4096)
            logger.info(f"⭕️ 서버 응답: {resp.decode()}")

            if tcp_stop_event :
                logger.info("🛑 Stopping TCP thread after payload transmission")
                tcp_stop_event.set()
    except Exception as e:
        logger.error(f"❌ Error during payload transfer: {e}")