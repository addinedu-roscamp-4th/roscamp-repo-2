# rtsp_server.py

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
import signal
import sys
import threading
import requests
import os
import subprocess

# GStreamer 초기화
Gst.init(None)

# RTSP 서버 설정
RTSP_IP    = "192.168.0.156"
RTSP_PORT  = 8554
STREAM_PATH= "/stream"
RTSP_URL   = f"rtsp://{RTSP_IP}:{RTSP_PORT}{STREAM_PATH}"

# 수신 서버 API
STREAM_API_BASE = f"http://{RTSP_IP}:8000/stream"

def notify_robo_add():
    try:
        resp = requests.post(f"{STREAM_API_BASE}/add", json={"url": RTSP_URL})
        print(f"[Sender] notify add → {resp.status_code} {resp.text}")
    except Exception as e:
        print(f"[Sender] ERROR notifying add: {e}")

def notify_robo_remove():
    try:
        resp = requests.post(f"{STREAM_API_BASE}/remove", json={"url": RTSP_URL})
        print(f"[Sender] notify remove → {resp.status_code} {resp.text}")
    except Exception as e:
        print(f"[Sender] ERROR notifying remove: {e}")

def shutdown(signum=None, frame=None):
    """녹화 중지 알림 후 프로세스 종료"""
    print(f"[Sender] shutting down RTSP server")
    notify_robo_remove()
    sys.exit(0)

class RTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, video_path):
        super().__init__()
        launch = (
            f'( uridecodebin uri=file://{video_path} '
            '! videoconvert '
            '! x264enc tune=zerolatency bitrate=500 speed-preset=superfast '
            '! rtph264pay name=pay0 pt=96 )'
        )
        self.set_launch(launch)
        self.set_shared(True)

def get_media_duration(video_file: str) -> float:
    """ffprobe를 호출해 비디오 길이(초)를 반환"""
    cmd = [
        "ffprobe", "-v", "error",
        "-show_entries", "format=duration",
        "-of", "default=noprint_wrappers=1:nokey=1",
        video_file
    ]
    out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
    return float(out.strip())

def start_rtsp_server(video_file: str):
    # 1) RTSP 서버 기동
    server = GstRtspServer.RTSPServer()
    mounts = server.get_mount_points()
    factory = RTSPMediaFactory(video_file)
    mounts.add_factory(STREAM_PATH, factory)
    server.attach(None)
    print(f"[Sender] RTSP 서버 실행 → {RTSP_URL}")

    # 2) 녹화 시작 알림
    notify_robo_add()

    # 3) 자동 종료 타이머: 파일 길이 + 1초 후 shutdown()
    try:
        duration = get_media_duration(video_file)
        print(f"[Sender] media duration: {duration:.1f}s, scheduling shutdown")
        threading.Timer(duration + 1.0, shutdown).start()
    except Exception as e:
        print(f"[Sender] duration probe failed: {e}")

    # 4) GLib MainLoop 실행
    loop = GLib.MainLoop()
    loop.run()

def main():
    # 사용할 비디오 파일 절대경로
    video_file = "/home/addinedu/dev_ws/recording_20250410_142223.mp4"
    if not os.path.isfile(video_file):
        print(f"Error: 파일을 찾을 수 없습니다: {video_file}")
        sys.exit(1)

    # SIGINT/SIGTERM 시 shutdown 호출
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    start_rtsp_server(video_file)

if __name__ == "__main__":
    main()
