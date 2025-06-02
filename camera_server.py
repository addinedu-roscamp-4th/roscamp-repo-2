#!/usr/bin/env python3
"""
임시 카메라 서버 - 웹캠을 RTSP 스트림으로 송출
GStreamer RTSP 서버를 사용하여 웹캠 스트림 제공
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

# GStreamer 초기화
Gst.init(None)

class WebcamRTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, device="/dev/video2"):
        super().__init__()
        self.device = device
        
        if device == "testsrc":
            launch = (
                '( videotestsrc pattern=smpte is-live=true '
                '! video/x-raw,width=640,height=480,framerate=30/1 '
                '! videoconvert '
                '! x264enc tune=zerolatency bitrate=1000 speed-preset=ultrafast '
                '! rtph264pay name=pay0 pt=96 )'
            )
        else:
            launch = (
                f'( v4l2src device={device} '
                '! video/x-raw,width=640,height=480,framerate=30/1 '
                '! videoconvert '
                '! x264enc tune=zerolatency bitrate=1000 speed-preset=ultrafast '
                '! rtph264pay name=pay0 pt=96 )'
            )
        
        print(f"🎥 GStreamer 파이프라인: {launch}")
        self.set_launch(launch)
        self.set_shared(True)

    def do_configure(self, rtsp_media):
        """RTSPMedia가 준비되었을 때 호출되는 안전한 콜백"""
        print("🔗 Media configured, setting up signal watch...")
        bus = rtsp_media.get_element().get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_message)

    def on_message(self, bus, message):
        """스트리밍 중 GStreamer 메시지 처리"""
        if message.type == Gst.MessageType.EOS:
            print("🛑 End of stream")
        elif message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"❌ GStreamer Error: {err} - {debug}")


class RTSPCameraServer:
    def __init__(self, device="/dev/video2", rtsp_port=8554, stream_path="/webcam"):
        self.device = device
        self.rtsp_port = rtsp_port
        self.stream_path = stream_path
        self.rtsp_url = f"rtsp://localhost:{rtsp_port}{stream_path}"
        self.server = None
        self.loop = None
        self.running = False
        
    def start_rtsp_server(self):
        """GStreamer RTSP 서버 시작"""
        print(f"🎥 웹캠 RTSP 서버 시작: {self.device} → {self.rtsp_url}")
        
        try:
            # RTSP 서버 생성
            self.server = GstRtspServer.RTSPServer()
            self.server.set_service(str(self.rtsp_port))
            
            # 마운트 포인트 설정
            mounts = self.server.get_mount_points()
            factory = WebcamRTSPMediaFactory(self.device)
            mounts.add_factory(self.stream_path, factory)
            
            # 서버 시작
            self.server.attach(None)
            print(f"✅ RTSP 서버 시작됨")
            print(f"📡 RTSP URL: {self.rtsp_url}")
            print(f"🔗 클라이언트 연결 대기 중...")
            
            self.running = True
            return True
            
        except Exception as e:
            print(f"❌ RTSP 서버 시작 실패: {e}")
            return False
    
    def run(self):
        """메인 루프 실행"""
        if not self.running:
            return False
            
        try:
            # GLib 메인 루프 시작
            self.loop = GLib.MainLoop()
            print("🔄 메인 루프 시작...")
            self.loop.run()
            
        except KeyboardInterrupt:
            print("\n📡 Ctrl+C 감지 - 서버 종료 중...")
            self.stop()
        except Exception as e:
            print(f"❌ 메인 루프 오류: {e}")
            self.stop()
    
    def stop(self):
        """RTSP 서버 중지"""
        print("🛑 RTSP 서버 중지 중...")
        self.running = False
        
        if self.loop and self.loop.is_running():
            self.loop.quit()
            
        print("✅ RTSP 서버 정상 종료")
    
    def status(self):
        """서버 상태 확인"""
        if self.running:
            return f"🟢 실행 중: {self.rtsp_url}"
        else:
            return "🔴 중지됨"
    
    def get_stream_stats(self):
        """현재 스트림 통계 정보"""
        if not self.running:
            return "🔴 스트림 중지됨"
        
        stats = f"""
📊 스트림 상태:
- RTSP URL: {self.rtsp_url}
- 디바이스: {self.device}
- 포트: {self.rtsp_port}
- 스트림 경로: {self.stream_path}
- 상태: 🟢 실행 중
        """
        return stats.strip()
    
    def test_rtsp_stream(self):
        """RTSP 스트림 테스트 (ffplay 사용)"""
        print(f"🧪 RTSP 스트림 테스트 시작: {self.rtsp_url}")
        print("ffplay로 스트림을 확인합니다...")
        
        test_cmd = ['ffplay', '-rtsp_transport', 'tcp', self.rtsp_url]
        
        try:
            subprocess.run(test_cmd, check=True)
        except FileNotFoundError:
            print("❌ ffplay가 설치되지 않았습니다. 설치해주세요:")
            print("sudo apt-get install ffmpeg")
        except subprocess.CalledProcessError as e:
            print(f"❌ ffplay 테스트 실패: {e}")
        except KeyboardInterrupt:
            print("🛑 테스트 중단됨")

def signal_handler(sig, frame):
    """시그널 핸들러"""
    print("\n📡 Ctrl+C 감지 - 서버 종료 중...")
    if 'server' in globals():
        server.stop()
    sys.exit(0)

def check_device_available(device):
    """디바이스 사용 가능 여부 확인"""
    if device == "testsrc":
        return True
        
    try:
        if not os.path.exists(device):
            print(f"❌ 디바이스가 존재하지 않습니다: {device}")
            return False
            
        # lsof로 디바이스 사용 중인지 확인
        result = subprocess.run(['lsof', device], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"⚠️ 디바이스가 사용 중입니다: {device}")
            print("사용 중인 프로세스:")
            print(result.stdout)
            
            # 사용 중이어도 GStreamer로는 접근 가능할 수 있음
            print("🔄 GStreamer로 접근을 시도합니다...")
            return True
            
        return True
        
    except Exception as e:
        print(f"디바이스 확인 중 오류: {e}")
        return True  # 오류가 있어도 시도해봄

def check_gstreamer_dependencies():
    """GStreamer 의존성 확인"""
    try:
        # GStreamer 플러그인 확인
        print("🔍 GStreamer 의존성 확인 중...")
        
        # v4l2 플러그인 확인
        result = subprocess.run(['gst-inspect-1.0', 'v4l2src'], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            print("❌ GStreamer v4l2 플러그인이 설치되지 않았습니다.")
            print("설치 명령어: sudo apt-get install gstreamer1.0-plugins-good")
            return False
            
        # x264 플러그인 확인
        result = subprocess.run(['gst-inspect-1.0', 'x264enc'], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            print("❌ GStreamer x264 플러그인이 설치되지 않았습니다.")
            print("설치 명령어: sudo apt-get install gstreamer1.0-plugins-ugly")
            return False
            
        print("✅ GStreamer 의존성 확인 완료")
        return True
        
    except FileNotFoundError:
        print("❌ GStreamer가 설치되지 않았습니다.")
        print("설치 명령어:")
        print("sudo apt-get install gstreamer1.0-tools")
        print("sudo apt-get install gstreamer1.0-plugins-base")
        print("sudo apt-get install gstreamer1.0-plugins-good")
        print("sudo apt-get install gstreamer1.0-plugins-ugly")
        print("sudo apt-get install python3-gi")
        return False
    except Exception as e:
        print(f"의존성 확인 중 오류: {e}")
        return True  # 오류가 있어도 시도해봄

def user_input_handler(server):
    """사용자 입력 처리 (별도 스레드)"""
    print("\n📋 명령어 옵션:")
    print("- 'status': 서버 상태 확인")
    print("- 'stats': 스트림 통계 정보") 
    print("- 'test': ffplay로 스트림 테스트")
    print("- 'quit': 서버 종료")
    print("- Ctrl+C: 서버 종료")
    
    try:
        while server.running:
            try:
                user_input = input("\n> ").strip().lower()
                if user_input in ['quit', 'exit', 'q']:
                    server.stop()
                    break
                elif user_input == 'status':
                    print(server.status())
                elif user_input == 'stats':
                    print(server.get_stream_stats())
                elif user_input == 'test':
                    print("📺 새 터미널에서 ffplay 테스트를 시작합니다...")
                    print(f"실행할 명령어: ffplay -rtsp_transport tcp {server.rtsp_url}")
                    # 백그라운드에서 테스트 실행
                    test_thread = threading.Thread(target=server.test_rtsp_stream, daemon=True)
                    test_thread.start()
                elif user_input == 'help':
                    print("사용 가능한 명령어: status, stats, test, quit, help")
                elif user_input:
                    print("알 수 없는 명령어입니다. 'help'를 입력하세요.")
            except (EOFError, OSError):
                # 입력이 불가능한 경우 (백그라운드 실행 등)
                print("⚠️ 사용자 입력 불가 - 백그라운드 모드로 실행 중")
                while server.running:
                    time.sleep(1)
                break
                    
    except EOFError:
        pass
    except KeyboardInterrupt:
        pass

def main():
    print("=" * 60)
    print("📡 임시 카메라 서버 (RTSP 송출)")
    print("=" * 60)
    
    # 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # GStreamer 의존성 확인
    if not check_gstreamer_dependencies():
        print("\n❌ 필요한 의존성이 설치되지 않았습니다.")
        return
    
    # 디바이스 확인
    device = "/dev/video2"
    if not check_device_available(device):
        print("📋 디바이스 선택 옵션:")
        print("1. 다른 디바이스 사용 (예: /dev/video0)")
        print("2. 테스트 패턴 사용")
        
        choice = input("선택하세요 (1/2): ").strip()
        if choice == "1":
            device = input("디바이스 경로 입력: ").strip()
        elif choice == "2":
            device = "testsrc"  # 테스트 패턴 사용
        else:
            print("❌ 잘못된 선택입니다.")
            return
    
    # 서버 생성 및 시작
    global server
    server = RTSPCameraServer(device=device)
    
    if server.start_rtsp_server():
        # 사용자 입력 처리 스레드 시작
        input_thread = threading.Thread(target=user_input_handler, args=(server,), daemon=True)
        input_thread.start()
        
        # 메인 루프 실행 (블로킹)
        server.run()
    else:
        print("❌ 서버 시작에 실패했습니다.")

if __name__ == "__main__":
    main() 