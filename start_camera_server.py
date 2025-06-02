#!/usr/bin/env python3
"""
카메라 서버 자동 시작 스크립트
백그라운드에서 실행되도록 설계됨
"""
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
import signal
import sys
import os
import subprocess
import threading
import time
import logging

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/tmp/camera_server.log'),
        logging.StreamHandler()
    ]
)

# GStreamer 초기화
Gst.init(None)

class WebcamRTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, device="/dev/video2"):
        super().__init__()
        self.device = device
        
        # GStreamer 파이프라인 구성
        if device == "testsrc":
            # 테스트 패턴 사용
            launch = (
                '( videotestsrc pattern=smpte is-live=true '
                '! video/x-raw,width=640,height=480,framerate=30/1 '
                '! videoconvert '
                '! x264enc tune=zerolatency bitrate=1000 speed-preset=ultrafast '
                '! rtph264pay name=pay0 pt=96 )'
            )
        else:
            # 실제 웹캠 사용
            launch = (
                f'( v4l2src device={device} '
                '! video/x-raw,width=640,height=480,framerate=30/1 '
                '! videoconvert '
                '! x264enc tune=zerolatency bitrate=1000 speed-preset=ultrafast '
                '! rtph264pay name=pay0 pt=96 )'
            )
        
        logging.info(f"🎥 GStreamer 파이프라인: {launch}")
        self.set_launch(launch)
        self.set_shared(True)
        
    def do_create_element(self, url):
        """미디어 엘리먼트 생성시 호출"""
        logging.info(f"📺 클라이언트 연결 요청: {url.get_request_uri()}")
        return super().do_create_element(url)

class AutoRTSPServer:
    def __init__(self, device="/dev/video2", rtsp_port=8554, stream_path="/webcam"):
        self.device = device
        self.rtsp_port = rtsp_port
        self.stream_path = stream_path
        self.rtsp_url = f"rtsp://localhost:{rtsp_port}{stream_path}"
        self.server = None
        self.loop = None
        self.running = False
        
    def start(self):
        """RTSP 서버 시작"""
        try:
            logging.info(f"🎥 자동 RTSP 서버 시작: {self.device} → {self.rtsp_url}")
            
            # RTSP 서버 생성
            self.server = GstRtspServer.RTSPServer()
            self.server.set_service(str(self.rtsp_port))
            
            # 마운트 포인트 설정
            mounts = self.server.get_mount_points()
            factory = WebcamRTSPMediaFactory(self.device)
            mounts.add_factory(self.stream_path, factory)
            
            # 서버 시작
            self.server.attach(None)
            self.running = True
            
            logging.info(f"✅ RTSP 서버 시작됨")
            logging.info(f"📡 RTSP URL: {self.rtsp_url}")
            
            # GLib 메인 루프 시작
            self.loop = GLib.MainLoop()
            logging.info("🔄 백그라운드 모드로 실행 중...")
            self.loop.run()
            
        except Exception as e:
            logging.error(f"❌ RTSP 서버 시작 실패: {e}")
            return False
    
    def stop(self):
        """RTSP 서버 중지"""
        logging.info("🛑 RTSP 서버 중지 중...")
        self.running = False
        
        if self.loop and self.loop.is_running():
            self.loop.quit()
            
        logging.info("✅ RTSP 서버 정상 종료")

def signal_handler(sig, frame):
    """시그널 핸들러"""
    logging.info("📡 종료 신호 수신 - 서버 종료 중...")
    if 'rtsp_server' in globals():
        rtsp_server.stop()
    sys.exit(0)

def check_device_available(device):
    """디바이스 사용 가능 여부 확인"""
    if device == "testsrc":
        return True
        
    if not os.path.exists(device):
        logging.warning(f"❌ 디바이스가 존재하지 않습니다: {device}")
        return False
        
    return True

def main():
    # 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    logging.info("=" * 60)
    logging.info("📡 자동 카메라 서버 (RTSP 송출)")
    logging.info("=" * 60)
    
    # 디바이스 확인 및 선택 - 테스트 패턴 우선 사용
    device = "/dev/video2"
    if not check_device_available(device):
        logging.info("🔄 테스트 패턴으로 대체합니다.")
        device = "testsrc"
    else:
        # 디바이스가 사용 가능하더라도 안정성을 위해 테스트 패턴 사용
        logging.info("🧪 안정성을 위해 테스트 패턴을 사용합니다.")
        device = "testsrc"
    
    # 서버 생성 및 시작
    global rtsp_server
    rtsp_server = AutoRTSPServer(device=device)
    rtsp_server.start()

if __name__ == "__main__":
    main() 