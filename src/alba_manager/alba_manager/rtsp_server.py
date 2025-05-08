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
from datetime import datetime, timezone, timedelta

# GStreamer 초기화
Gst.init(None)

# RTSP 서버 설정
RTSP_IP    = "192.168.0.156"
RTSP_PORT  = 8554
STREAM_PATH= "/stream"
RTSP_URL   = f"rtsp://{RTSP_IP}:{RTSP_PORT}{STREAM_PATH}"
video_file = "/home/addinedu/dev_ws/recording_20250410_142223.mp4"


# 수신 서버 API
STREAM_API_BASE = f"http://{RTSP_IP}:8000/api/video-streams"

# 이 변수는 한 번만 녹화 정보를 보내도록 제어
has_notified = False

def notify_robo_add():
    try:
        duration = get_media_duration(video_file)
        duration_timedelta = timedelta(seconds=duration)

        resp = requests.post(f"{STREAM_API_BASE}/add", json={
            "url": RTSP_URL, 
            "source_type": "PINKY", 
            "source_id": "rtsp_server",
            "recording_path": video_file,
            "recording_started_at": (datetime.now(timezone.utc) - duration_timedelta).isoformat(),
            "recording_ended_at": datetime.utcnow().isoformat() + "Z",
            })
        print(f"[Sender] notify add → {resp.status_code} {resp.text}")
    except Exception as e:
        print(f"[Sender] ERROR notifying add: {e}")

def notify_robo_remove():
    try:
        # 스트리밍 종료 요청
        resp = requests.post(f"{STREAM_API_BASE}/remove", json={"url": RTSP_URL})
        print(f"[Sender] notify remove → {resp.status_code} {resp.text}")
    except Exception as e:
        print(f"[Sender] ERROR notifying remove: {e}")


class RTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, video_path, shutdown_callback):
        super().__init__()
        self.shutdown_callback = shutdown_callback
        launch = (
            f'( uridecodebin uri=file://{video_path} '
            '! videoconvert '
            '! x264enc tune=zerolatency bitrate=500 speed-preset=superfast '
            '! rtph264pay name=pay0 pt=96 )'
        )
        self.set_launch(launch)
        self.set_shared(True)

    def on_media_ready(self, media):
        """스트리밍이 시작되면 호출되는 콜백"""
        print("[Sender] Media ready, streaming started")
        media.get_bus().add_signal_watch()
        media.get_bus().connect("message", self.on_message)

    def on_message(self, bus, message):
        """EOS 이벤트가 발생하면 종료"""
        if message.type == Gst.MessageType.EOS:
            print("[Sender] End of stream reached, shutting down...")
            self.shutdown_callback()

def shutdown(signum=None, frame=None):
    """녹화 중지 알림 후 프로세스 종료"""
    global has_notified
    if has_notified:
        return  # 이미 녹화 정보를 보냈다면 중복으로 실행되지 않도록 방지
    
    print(f"[Sender] shutting down RTSP server")
    notify_robo_remove()  # 종료 후 remove 요청 보내기
    sys.exit(0)


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
    factory = RTSPMediaFactory(video_file, shutdown)
    mounts.add_factory(STREAM_PATH, factory)
    server.attach(None)
    print(f"[Sender] RTSP 서버 실행 → {RTSP_URL}")

    # 2) 녹화 시작 알림
    notify_robo_add()

    # 3) GLib MainLoop 실행
    loop = GLib.MainLoop()
    loop.run()

def main():
    # 사용할 비디오 파일 절대경로
    if not os.path.isfile(video_file):
        print(f"Error: 파일을 찾을 수 없습니다: {video_file}")
        sys.exit(1)

    # SIGINT/SIGTERM 시 shutdown 호출
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    start_rtsp_server(video_file)

if __name__ == "__main__":
    main()
