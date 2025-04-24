#!/usr/bin/env python3
import cv2
import socket
import struct
import argparse
import time
import sys
import os
from collections import deque
from datetime import datetime
import numpy as np

def make_output_path(out_arg):
    # out_arg가 디렉터리라면 timestamped filename 생성
    if os.path.isdir(out_arg):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = f"recorded_{ts}.avi"
        return os.path.join(out_arg, fname)
    # 그렇지 않으면 파일 경로로 간주
    return out_arg

def main():
    parser = argparse.ArgumentParser(
        description="UDP Video Receiver with Recording & Latency (custom save location)"
    )
    parser.add_argument('--port',   type=int,   default=5000,
                        help='Listen port')
    parser.add_argument('--output', default='.',
                        help='Recording path. '
                             'If a directory, creates timestamped file inside. '
                             'If a file, uses that path directly.')
    parser.add_argument('--fps',    type=float, default=20.0,
                        help='Expected FPS (for VideoWriter)')
    parser.add_argument('--width',  type=int,   default=640,
                        help='Frame width')
    parser.add_argument('--height', type=int,   default=480,
                        help='Frame height')
    args = parser.parse_args()

    # UDP 소켓 설정
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', args.port))
    sock.settimeout(1.0)

    print(f"▶ Listening on udp://0.0.0.0:{args.port}")
    print("   Press 'r' to toggle recording, 'q' to quit.")

    recording = False
    writer = None
    output_path = None

    latencies = deque(maxlen=1000)
    font = cv2.FONT_HERSHEY_SIMPLEX

    try:
        while True:
            try:
                packet, addr = sock.recvfrom(65536)
            except socket.timeout:
                continue

            if len(packet) <= 8:
                continue
            # 송신 timestmap 언패킹
            ts_sent = struct.unpack('d', packet[:8])[0]
            jpeg_data = packet[8:]

            # 레이턴시 계산
            ts_recv = time.time()
            latency_ms = (ts_recv - ts_sent) * 1000.0
            latencies.append(latency_ms)
            avg_latency = sum(latencies) / len(latencies)

            # JPEG → BGR
            npdata = np.frombuffer(jpeg_data, dtype=np.uint8)
            frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # 오버레이
            cv2.putText(frame,
                        f"Latency: {latency_ms:.1f} ms  Avg: {avg_latency:.1f} ms",
                        (10, 25), font, 0.7, (0,255,0), 2)

            cv2.imshow('UDP Video', frame)

            # 녹화 중일 때
            if recording and writer:
                writer.write(frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):
                recording = not recording
                if recording:
                    # 녹화 시작: output_path 결정, VideoWriter 오픈
                    output_path = make_output_path(args.output)
                    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    writer = cv2.VideoWriter(
                        output_path, fourcc, args.fps,
                        (args.width, args.height)
                    )
                    print(f"▶ Recording started → {output_path}")
                else:
                    # 녹화 종료
                    writer.release()
                    writer = None
                    print(f"▶ Recording stopped (saved to {output_path})")
            elif key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        if writer:
            writer.release()
        sock.close()
        cv2.destroyAllWindows()
        print("▶ Receiver exited")

if __name__ == '__main__':
    main()
