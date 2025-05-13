#!/usr/bin/env python3
import cv2
import socket
import argparse
import time
import sys
import struct

def main():
    parser = argparse.ArgumentParser(description="UDP Video Sender (using default OpenCV backend)")
    parser.add_argument('--host',    default='192.168.0.156', help='Destination IP')
    parser.add_argument('--port',    type=int, default=5000, help='Destination port')
    parser.add_argument('--device',  default='/dev/jetcocam0', help='Video device path (e.g. /dev/jetcocam0)')
    parser.add_argument('--width',   type=int, default=640,    help='Capture width')
    parser.add_argument('--height',  type=int, default=480,    help='Capture height')
    parser.add_argument('--fps',     type=int, default=20,     help='Frames per second')
    parser.add_argument('--quality', type=int, default=80,     help='JPEG quality (0-100)')
    args = parser.parse_args()

    # 기본 백엔드로 카메라 열기
    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS,          args.fps)

    if not cap.isOpened():
        print(f"[ERROR] cannot open camera: {args.device}", file=sys.stderr)
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)
    interval = 1.0 / args.fps

    print(f"▶ Streaming to udp://{args.host}:{args.port} at {args.fps} FPS")
    try:
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            if not ret or frame is None:
                print("! cannot read frame")
                time.sleep(0.1)
                continue
            # JPEG 인코딩
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
            if not ok:
                print("! JPEG encode failed")
                continue
            # 현재 시간(초 단위, double) 을 8바이트로 패킹 → 프레임 데이터 앞에 붙여 보냄
            ts = time.time()
            pkt = struct.pack('d', ts) + buf.tobytes()
            sock.sendto(pkt, addr)
            # FPS 유지
            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        sock.close()
        print("▶ Sender stopped.")

if __name__ == '__main__':
    main()
