#UDP_server.py

#!/usr/bin/env python3
import cv2
import socket
import argparse
import time
import sys
import struct

def main():
    # === 사용자 지정 파라미터 ===
    DEST_IP = '192.168.0.189'    # PC의 IP 주소
    DEST_PORT = 5001             # robotb4용 포트 번호
    DEVICE_PATH = '/dev/jetcocam0'  # 로봇 카메라 경로
    WIDTH = 640
    HEIGHT = 480
    FPS = 20
    JPEG_QUALITY = 80

    # === 카메라 열기 ===
    cap = cv2.VideoCapture(DEVICE_PATH)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          FPS)

    if not cap.isOpened():
        print(f"[ERROR] cannot open camera: {DEVICE_PATH}", file=sys.stderr)
        sys.exit(1)

    # === UDP 소켓 생성 및 목적지 설정 ===
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (DEST_IP, DEST_PORT)
    interval = 1.0 / FPS

    print(f"📤 스트리밍 시작: udp://{DEST_IP}:{DEST_PORT} @ {FPS} FPS")
    try:
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            if not ret or frame is None:
                print("! 프레임 읽기 실패")
                time.sleep(0.1)
                continue

            # JPEG 압축
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ok:
                print("! JPEG 인코딩 실패")
                continue

            # 현재 시간 정보를 앞에 붙여서 송신
            ts = time.time()
            pkt = struct.pack('d', ts) + buf.tobytes()
            sock.sendto(pkt, addr)

            # 설정된 FPS에 맞춰 슬립
            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("\n📴 송신 종료됨.")
    finally:
        cap.release()
        sock.close()

if __name__ == '__main__':
    main()
