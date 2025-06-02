# robodine_service/backend/app/routes/live_streaming.py
import asyncio
import logging
import os
import json
import uuid
import subprocess
from typing import Dict, Optional, List, Tuple
import time
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
import threading
from collections import defaultdict, deque
import socket
import struct
import cv2
import numpy as np
from threading import Thread, Event
import queue

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException, Depends
from starlette.responses import JSONResponse
from sqlalchemy.orm import Session

# aiortc 라이브러리 import 추가
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer, MediaRelay
from av import VideoFrame

from app.core.db_config import get_db
# streaming.py에서 비디오 레코더 기능 import
from app.routes.streaming import initialize_video_recorder, get_video_recorder

router = APIRouter()
logger = logging.getLogger("robodine.live_streaming")
logger.setLevel(logging.DEBUG)

# 성능 측정을 위한 클래스
class StreamingPerformanceMonitor:
    """스트리밍 성능 측정 및 모니터링 클래스"""
    
    def __init__(self):
        self.frame_timestamps = deque(maxlen=100)  # 최근 100개 프레임 타임스탬프
        self.processing_times = deque(maxlen=50)   # 최근 50개 처리 시간
        self.webrtc_metrics = deque(maxlen=30)     # 최근 30개 WebRTC 메트릭
        self.last_performance_log = time.time()
        self.frame_count = 0
        
    def record_frame_received(self, timestamp):
        """UDP 프레임 수신 시간 기록"""
        self.frame_timestamps.append(timestamp)
        self.frame_count += 1
        
    def record_processing_time(self, process_name, start_time, end_time):
        """처리 시간 기록"""
        processing_time = (end_time - start_time) * 1000  # 밀리초 단위
        self.processing_times.append({
            'process': process_name,
            'time_ms': processing_time,
            'timestamp': end_time
        })
        
    def record_webrtc_metric(self, metric_name, value, session_id=None):
        """WebRTC 메트릭 기록"""
        self.webrtc_metrics.append({
            'metric': metric_name,
            'value': value,
            'session_id': session_id,
            'timestamp': time.time()
        })
        
    def calculate_fps(self):
        """현재 FPS 계산"""
        if len(self.frame_timestamps) < 2:
            return 0
        
        time_span = self.frame_timestamps[-1] - self.frame_timestamps[0]
        if time_span <= 0:
            return 0
            
        return (len(self.frame_timestamps) - 1) / time_span
        
    def calculate_average_latency(self):
        """평균 지연 시간 계산"""
        if not self.processing_times:
            return 0
            
        recent_times = [p['time_ms'] for p in self.processing_times if p['process'] == 'frame_processing']
        if not recent_times:
            return 0
            
        return sum(recent_times) / len(recent_times)
        
    def log_performance_summary(self):
        """성능 요약 로그 출력"""
        now = time.time()
        
        # 5초마다 성능 요약 출력
        if now - self.last_performance_log >= 5.0:
            fps = self.calculate_fps()
            avg_latency = self.calculate_average_latency()
            
            logger.info(f"=== 스트리밍 성능 요약 ===")
            logger.info(f"현재 FPS: {fps:.2f}")
            logger.info(f"총 프레임 수: {self.frame_count}")
            logger.info(f"평균 프레임 처리 시간: {avg_latency:.2f}ms")
            
            if self.processing_times:
                recent_processing = list(self.processing_times)[-10:]  # 최근 10개
                logger.info(f"최근 처리 시간들: {[f'{p['process']}: {p['time_ms']:.1f}ms' for p in recent_processing]}")
                
            if self.webrtc_metrics:
                recent_metrics = list(self.webrtc_metrics)[-5:]  # 최근 5개
                logger.info(f"최근 WebRTC 메트릭: {[f'{m['metric']}: {m['value']}' for m in recent_metrics]}")
                
            self.last_performance_log = now

# 전역 성능 모니터 인스턴스
performance_monitor = StreamingPerformanceMonitor()

# UDP 비디오 스트림 수신기 클래스
class UDPReceiver:
    def __init__(self, port=5000):
        self.port = port
        self.running = False
        self.socket = None
        self.latest_frame = None
        self.frame_count = 0
        self.last_update_time = time.time()
        self.lock = threading.Lock()
        self.receiver_thread = None
        
    def start(self):
        """UDP 수신 시작"""
        if self.running:
            return
            
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(('', self.port))
            self.socket.settimeout(1.0)  # 1초 타임아웃
            self.running = True
            
            self.receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receiver_thread.start()
            logger.info(f"UDP 수신기 시작됨 (포트: {self.port})")
        except Exception as e:
            logger.error(f"UDP 수신기 시작 실패: {e}")
            self.running = False
    
    def stop(self):
        """UDP 수신 중지"""
        self.running = False
        if self.socket:
            self.socket.close()
        if self.receiver_thread:
            self.receiver_thread.join(timeout=2.0)
        logger.info("UDP 수신기 중지됨")
    
    def has_frames(self):
        """프레임이 있는지 확인 (최근 5초 이내)"""
        with self.lock:
            current_time = time.time()
            return (self.latest_frame is not None and 
                   current_time - self.last_update_time < 5.0 and
                   self.frame_count > 0)
    
    def get_latest_frame(self):
        """가장 최근 프레임 반환"""
        with self.lock:
            return self.latest_frame
    
    def _receive_loop(self):
        """UDP 데이터 수신 루프"""
        buffer = b""
        
        while self.running:
            try:
                receive_start_time = time.time()
                data, addr = self.socket.recvfrom(65536)
                buffer += data
                
                # 성능 측정: UDP 수신 시간
                udp_receive_time = (time.time() - receive_start_time) * 1000
                if udp_receive_time > 10:  # 10ms 이상인 경우만 로깅
                    logger.debug(f"UDP 수신 지연: {udp_receive_time:.1f}ms")
                
                # 완전한 프레임 찾기 (타임스탬프 + JPEG 마커)
                while True:
                    # 타임스탬프 찾기 (8바이트)
                    if len(buffer) < 8:
                        break
                    
                    timestamp_bytes = buffer[:8]
                    frame_timestamp = struct.unpack('>d', timestamp_bytes)[0]
                    
                    # JPEG 시작 마커 찾기
                    jpeg_start = buffer.find(b'\xff\xd8', 8)
                    if jpeg_start == -1:
                        break
                    
                    # JPEG 끝 마커 찾기  
                    jpeg_end = buffer.find(b'\xff\xd9', jpeg_start)
                    if jpeg_end == -1:
                        break
                    
                    # 완전한 JPEG 프레임 추출
                    jpeg_data = buffer[jpeg_start:jpeg_end + 2]
                    
                    # 성능 측정: 프레임 처리 시작
                    frame_process_start = time.time()
                    
                    # OpenCV로 디코딩
                    try:
                        frame_array = np.frombuffer(jpeg_data, np.uint8)
                        frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)
                        
                        if frame is not None:
                            with self.lock:
                                self.latest_frame = frame
                                self.frame_count += 1
                                self.last_update_time = time.time()
                                
                            # 성능 측정: 프레임 처리 완료
                            frame_process_end = time.time()
                            
                            # 프레임 처리 시간 기록
                            performance_monitor.record_processing_time(
                                'frame_processing', 
                                frame_process_start, 
                                frame_process_end
                            )
                            
                            # 프레임 수신 시간 기록
                            performance_monitor.record_frame_received(time.time())
                            
                            # 전체 지연 시간 계산 (UDP 프레임 타임스탬프 vs 현재 시간)
                            current_time = time.time()
                            if frame_timestamp > 0:
                                total_latency = (current_time - frame_timestamp) * 1000
                                performance_monitor.record_webrtc_metric('udp_to_decode_latency_ms', total_latency)
                                
                                if total_latency > 100:  # 100ms 이상 지연 시 경고
                                    logger.warning(f"높은 UDP 지연 감지: {total_latency:.1f}ms")
                            
                            # 주기적 성능 요약 출력
                            performance_monitor.log_performance_summary()
                                
                            if self.frame_count % 30 == 0:  # 30프레임마다 로그
                                fps = performance_monitor.calculate_fps()
                                avg_latency = performance_monitor.calculate_average_latency()
                                logger.info(f"UDP 프레임 수신: {self.frame_count}, FPS: {fps:.1f}, 평균 처리시간: {avg_latency:.1f}ms")
                    except Exception as e:
                        logger.warning(f"프레임 디코딩 실패: {e}")
                    
                    # 처리된 데이터 제거
                    buffer = buffer[jpeg_end + 2:]
                    
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.error(f"UDP 수신 오류: {e}")
                break

# 커스텀 비디오 스트림 트랙 클래스
class UDPVideoStreamTrack(VideoStreamTrack):
    """UDP로 받은 비디오를 WebRTC로 전송하는 트랙"""
    
    def __init__(self, udp_receiver):
        super().__init__()
        self.udp_receiver = udp_receiver
        self.pts = 0
        self.time_base = 1/30  # 30 FPS
        self.last_frame_time = time.time()
        
    async def recv(self):
        """비디오 프레임 생성"""
        webrtc_start_time = time.time()
        
        pts, time_base = await self.next_timestamp()
        
        # UDP에서 최신 프레임 가져오기
        frame = self.udp_receiver.get_latest_frame()
        
        if frame is None:
            # 프레임이 없으면 검은 화면 생성
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            performance_monitor.record_webrtc_metric('webrtc_no_frame_count', 1)
        else:
            # WebRTC 프레임 전송 간격 측정
            current_time = time.time()
            frame_interval = (current_time - self.last_frame_time) * 1000
            performance_monitor.record_webrtc_metric('webrtc_frame_interval_ms', frame_interval)
            self.last_frame_time = current_time
            
        # OpenCV BGR을 RGB로 변환
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # VideoFrame 생성
        video_frame = VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        
        # 성능 측정: WebRTC 프레임 처리 시간
        webrtc_end_time = time.time()
        webrtc_processing_time = (webrtc_end_time - webrtc_start_time) * 1000
        performance_monitor.record_processing_time('webrtc_frame_processing', webrtc_start_time, webrtc_end_time)
        
        if webrtc_processing_time > 50:  # 50ms 이상인 경우 경고
            logger.warning(f"WebRTC 프레임 처리 지연: {webrtc_processing_time:.1f}ms")
        
        return video_frame

# 전역 UDP 수신기 인스턴스
udp_receiver = UDPReceiver(port=5000)

# 서버 시작 시 UDP 수신기 시작
def start_udp_receiver():
    """UDP 수신기 및 비디오 레코더 시작"""
    udp_receiver.start()
    
    # 비디오 레코더 초기화 (데이터베이스 세션 팩토리 전달)
    try:
        initialize_video_recorder(udp_receiver, get_db)
        logger.info("UDP 수신기 및 비디오 레코더 초기화 완료")
    except Exception as e:
        logger.error(f"비디오 레코더 초기화 실패: {e}")

# 서버 종료 시 UDP 수신기 중지
def stop_udp_receiver():
    """UDP 수신기 및 비디오 레코더 중지"""
    udp_receiver.stop()
    
    # 비디오 레코더 중지
    try:
        video_recorder = get_video_recorder()
        if video_recorder:
            video_recorder.stop_recording()
            logger.info("비디오 레코더 중지 완료")
    except Exception as e:
        logger.error(f"비디오 레코더 중지 실패: {e}")

# 애플리케이션 시작 시 UDP 수신기 자동 시작
start_udp_receiver()

# 전역 리소스 관리
class DeviceResourceManager:
    def __init__(self):
        self._device_locks = {}  # 디바이스별 락
        self._device_users = {}  # 디바이스 사용자 추적
        self._session_timestamps = defaultdict(list)  # 세션별 요청 타임스탬프
        self._global_lock = threading.Lock()
        
    def acquire_device(self, device_path: str, session_id: str) -> bool:
        """디바이스 사용 권한 획득 (UDP 스트림의 경우 항상 True 반환)"""
        with self._global_lock:
            # UDP 스트림은 여러 세션에서 동시 접근 가능
            if device_path == 'udp_stream':
                logger.info(f"UDP 스트림 접근 허용 (세션: {session_id})")
                return True
                
            # 이미 다른 세션에서 사용 중인지 확인
            if device_path in self._device_users:
                current_user = self._device_users[device_path]
                if current_user != session_id:
                    logger.warning(f"디바이스 {device_path}가 세션 {current_user}에서 사용 중 (요청: {session_id})")
                    return False
                else:
                    # 같은 세션이면 재사용 허용
                    logger.info(f"디바이스 {device_path} 재사용 허용 (세션: {session_id})")
                    return True
            
            # 디바이스 사용 등록
            self._device_users[device_path] = session_id
            logger.info(f"디바이스 {device_path} 사용 시작 (세션: {session_id})")
            return True
    
    def release_device(self, device_path: str, session_id: str):
        """디바이스 사용 권한 해제"""
        with self._global_lock:
            # UDP 스트림은 해제할 필요 없음
            if device_path == 'udp_stream':
                return
                
            if device_path in self._device_users and self._device_users[device_path] == session_id:
                del self._device_users[device_path]
                logger.info(f"디바이스 {device_path} 사용 종료 (세션: {session_id})")
    
    def is_rate_limited(self, client_key: str, max_requests: int = 5, time_window: int = 10) -> bool:
        """Rate limiting 체크 (완화된 정책)"""
        now = time.time()
        timestamps = self._session_timestamps[client_key]
        
        # 시간 윈도우 밖의 기록 제거
        timestamps[:] = [ts for ts in timestamps if now - ts < time_window]
        
        # 요청 수 제한 체크
        if len(timestamps) >= max_requests:
            logger.warning(f"Rate limit 초과: {client_key} ({len(timestamps)}/{max_requests} in {time_window}s)")
            return True
        
        # 새 요청 기록
        timestamps.append(now)
        return False
    
    def get_client_session_count(self, client_key: str) -> int:
        """클라이언트의 활성 세션 수 조회"""
        session_count = 0
        for device_path, session_id in self._device_users.items():
            if client_key in session_id:  # 세션 ID에 클라이언트 키가 포함된 경우
                session_count += 1
        return session_count
    
    def cleanup_expired_sessions(self):
        """만료된 세션 정리"""
        now = time.time()
        expired_clients = []
        
        # 10분 이상 된 세션 타임스탬프 정리
        for client_key, timestamps in self._session_timestamps.items():
            timestamps[:] = [ts for ts in timestamps if now - ts < 600]  # 10분
            if not timestamps:
                expired_clients.append(client_key)
        
        # 빈 타임스탬프 목록 제거
        for client_key in expired_clients:
            del self._session_timestamps[client_key]
            
        logger.debug(f"만료된 세션 {len(expired_clients)}개 정리됨")

# 전역 리소스 매니저 인스턴스
resource_manager = DeviceResourceManager()

# —————————————
# 로그 파일 핸들러 설정 (logs/live_streaming.log)
# —————————————
log_dir = os.path.join(os.getcwd(), "logs")
os.makedirs(log_dir, exist_ok=True)
file_handler = logging.FileHandler(os.path.join(log_dir, "live_streaming.log"))
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(logging.Formatter(
    "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
))
logger.addHandler(file_handler)
# —————————————

# 고정 웹캠 설정 (UDP 스트림으로 변경)
DEFAULT_WEBCAMS = [
    {"id": "camera_2", "path": "udp_stream", "display_name": "UDP 웹캠 스트림"},
]

# 고정 RTSP 스트림 설정
DEFAULT_RTSP_STREAMS = [
    # {"id": "rtsp_main", "url": "rtsp://admin:admin123@192.168.0.108:554/live", "display_name": "메인 CCTV"},
    # {"id": "rtsp_entrance", "url": "rtsp://admin:admin123@192.168.0.109:554/live", "display_name": "입구 CCTV"},
]

# 스트림 및 클라이언트 관리를 위한 전역 변수
streams = {}
client_register = {}
webrtc_sessions = {}  # WebRTC 세션 관리

# 스트림 초기화 함수
def initialize_streams():
    """스트림 목록 초기화"""
    global streams
    streams.clear()
    
    # 웹캠 등록 (UDP 스트림)
    for webcam in DEFAULT_WEBCAMS:
        webcam_id = webcam["id"]
        streams[webcam_id] = {
            "id": webcam_id,
            "type": "webcam",
            "path": webcam["path"],
            "display_name": webcam.get("display_name", f"카메라 {webcam_id}"),
            "status": "ready",
            "client_count": 0
        }
        
        # UDP 스트림 상태 확인
        if udp_receiver.has_frames():
            logging.info(f"UDP 웹캠 등록: {webcam_id}, UDP 스트림 활성화됨")
        else:
            logging.warning(f"UDP 웹캠 등록: {webcam_id}, UDP 스트림 대기 중")
    
    # RTSP 스트림 등록
    for rtsp in DEFAULT_RTSP_STREAMS:
        rtsp_id = rtsp["id"]
        streams[rtsp_id] = {
            "id": rtsp_id,
            "type": "rtsp",
            "url": rtsp["url"],
            "display_name": rtsp.get("display_name", f"RTSP 스트림 {rtsp_id}"),
            "status": "ready",
            "client_count": 0
        }
        logging.info(f"RTSP 스트림 등록: {rtsp_id}, URL: {rtsp['url']}")
    
    return streams

# 초기화 실행
initialize_streams()

# WebRTC 연결 관리를 위한 클래스
class RTCSessionManager:
    def __init__(self):
        self.sessions: Dict[str, Dict] = {}
        self.clients: Dict[str, WebSocket] = {}
        self.webcam_streams: Dict[str, Dict] = {}
        self.rtsp_streams: Dict[str, Dict] = {}
        
        # 사용 가능한 카메라 초기화
        self._init_devices()
        
    def _init_devices(self):
        """초기 디바이스 설정"""
        # 기존 등록된 웹캠 및 RTSP 스트림 초기화
        self.webcam_streams.clear()
        self.rtsp_streams.clear()
        
        # 고정 웹캠 설정 등록 - 디바이스 존재 여부 확인 없이 강제 등록
        for webcam in DEFAULT_WEBCAMS:
            self.force_register_webcam(webcam["id"], webcam["path"], webcam.get("display_name"))
                
        # 미리 정의된 RTSP 스트림 등록
        for stream in DEFAULT_RTSP_STREAMS:
            self.register_rtsp_stream(stream["id"], stream["url"], stream.get("display_name"))
        
    async def create_session(self, client_id: str = None) -> str:
        """새 WebRTC 세션 생성"""
        session_id = str(uuid.uuid4())
        if not client_id:
            client_id = session_id
            
        self.sessions[session_id] = {
            "id": session_id,
            "client_id": client_id,
            "created_at": asyncio.get_event_loop().time(),
            "ice_candidates": [],
            "offer": None,
            "answer": None,
            "status": "created"
        }
        logger.info(f"세션 생성: {session_id}")
        return session_id
    
    def get_session(self, session_id: str) -> Optional[Dict]:
        """세션 정보 조회"""
        return self.sessions.get(session_id)
    
    def delete_session(self, session_id: str) -> bool:
        """세션 삭제"""
        if session_id in self.sessions:
            del self.sessions[session_id]
            logger.info(f"세션 삭제: {session_id}")
            return True
        return False

    async def register_client(self, client_id: str, websocket: WebSocket) -> None:
        """클라이언트 웹소켓 등록"""
        self.clients[client_id] = websocket
        logger.info(f"클라이언트 등록: {client_id}")
    
    async def unregister_client(self, client_id: str) -> None:
        """클라이언트 웹소켓 해제"""
        if client_id in self.clients:
            del self.clients[client_id]
            logger.info(f"클라이언트 해제: {client_id}")
    
    def get_client(self, client_id: str) -> Optional[WebSocket]:
        """클라이언트 조회"""
        return self.clients.get(client_id)
    
    def register_webcam(self, device_id: str, device_path: str, display_name: str = None) -> None:
        """웹캠 등록 (디바이스 존재 확인)"""
        # 디바이스 존재 여부 확인
        if not os.path.exists(device_path):
            logger.warning(f"웹캠 등록 실패: {device_path} 디바이스가 존재하지 않습니다")
            return

        self.webcam_streams[device_id] = {
            "id": device_id,
            "device_path": device_path,
            "status": "ready",
            "clients": set(),
            "display_name": display_name or f"카메라 {device_path.split('/')[-1]}"
        }
        logger.info(f"웹캠 등록: {device_id}, 경로: {device_path}")
    
    def force_register_webcam(self, device_id: str, device_path: str, display_name: str = None) -> None:
        """웹캠 강제 등록 (디바이스 존재 여부 무시)"""
        self.webcam_streams[device_id] = {
            "id": device_id,
            "device_path": device_path,
            "status": "ready",
            "clients": set(),
            "display_name": display_name or f"카메라 {device_path.split('/')[-1]}"
        }
        if not os.path.exists(device_path):
            logger.warning(f"웹캠 강제 등록 (디바이스 없음): {device_id}, 경로: {device_path}")
        else:
            logger.info(f"웹캠 강제 등록: {device_id}, 경로: {device_path}")
    
    def register_rtsp_stream(self, stream_id: str, rtsp_url: str, display_name: str = None) -> None:
        """RTSP 스트림 등록"""
        # URL 형식 검증
        if not rtsp_url.startswith("rtsp://"):
            logger.warning(f"잘못된 RTSP URL 형식: {rtsp_url}")
            return

        self.rtsp_streams[stream_id] = {
            "id": stream_id,
            "rtsp_url": rtsp_url,
            "status": "ready",
            "clients": set(),
            "display_name": display_name or f"RTSP 스트림 {stream_id.split('_')[-1] if '_' in stream_id else stream_id}"
        }
        logger.info(f"RTSP 스트림 등록: {stream_id}, URL: {rtsp_url}")
        
    def check_stream_exists(self, stream_id: str) -> bool:
        """스트림 ID가 존재하는지 확인"""
        return stream_id in self.webcam_streams or stream_id in self.rtsp_streams
    
    def get_stream_info(self, stream_id: str) -> Optional[Dict]:
        """스트림 정보 조회"""
        if stream_id in self.webcam_streams:
            stream = self.webcam_streams[stream_id]
            return {
                "id": stream_id,
                "type": "webcam",
                "path": stream["device_path"],
                "status": stream["status"],
                "display_name": stream["display_name"]
            }
        elif stream_id in self.rtsp_streams:
            stream = self.rtsp_streams[stream_id]
            return {
                "id": stream_id,
                "type": "rtsp",
                "url": stream["rtsp_url"],
                "status": stream["status"],
                "display_name": stream["display_name"]
            }
        return None
    
    def get_available_streams(self) -> List[Dict]:
        """사용 가능한 모든 스트림 목록 반환"""
        streams = []
        
        # 웹캠 스트림 추가
        for device_id, info in self.webcam_streams.items():
            streams.append({
                "id": device_id,
                "type": "webcam",
                "path": info["device_path"],
                "status": info["status"],
                "client_count": len(info["clients"]),
                "display_name": info["display_name"]
            })
        
        # RTSP 스트림 추가
        for stream_id, info in self.rtsp_streams.items():
            streams.append({
                "id": stream_id,
                "type": "rtsp",
                "url": info["rtsp_url"],
                "status": info["status"],
                "client_count": len(info["clients"]),
                "display_name": info["display_name"]
            })
            
        return streams
        
    def refresh_devices(self):
        """사용 가능한 디바이스 새로고침"""
        self._init_devices()
        return self.get_available_streams()
        
    def update_stream_config(self):
        """현재 스트림 설정을 저장"""
        logger.info("스트림 설정 업데이트 요청")
        return True

# 글로벌 RTC 세션 관리자 인스턴스
rtc_manager = RTCSessionManager()

# find_by_id_in_dict 함수 추가
def find_by_id_in_dict(items_dict: Dict, item_id: str) -> Optional[Dict]:
    """
    ID로 사전에서 항목을 찾아 반환합니다.
    
    Args:
        items_dict: 검색할 사전
        item_id: 찾을 항목의 ID
        
    Returns:
        찾은 항목 또는 None
    """
    return items_dict.get(item_id)

@router.get("/streams")
async def list_streams(db: Session = Depends(get_db)):
    """사용 가능한 모든 스트림 목록 반환 (실시간 스트림 + 녹화된 영상)"""
    # 실시간 스트림 목록을 리스트로 변환
    stream_list = []
    for stream_id, stream_data in streams.items():
        stream_list.append(stream_data)
    
    # 녹화된 영상들을 스트림 목록에 추가
    try:
        from app.models.video_stream import VideoStream
        from app.models.enums import StreamSourceType
        
        recordings = db.query(VideoStream).filter(
            VideoStream.source_type == StreamSourceType.WEBCAM
        ).order_by(VideoStream.recording_started_at.desc()).limit(20).all()  # 최근 20개만
        
        for recording in recordings:
            if recording.recording_path and os.path.exists(recording.recording_path):
                stream_list.append({
                    "id": f"recording_{recording.id}",
                    "type": "recording",
                    "path": recording.recording_path,
                    "url": recording.url,
                    "display_name": f"녹화영상 {recording.recording_started_at.strftime('%Y-%m-%d %H:%M')}",
                    "status": "recorded",
                    "client_count": 0,
                    "recording_started_at": recording.recording_started_at.isoformat() if recording.recording_started_at else None,
                    "recording_ended_at": recording.recording_ended_at.isoformat() if recording.recording_ended_at else None,
                    "file_size": os.path.getsize(recording.recording_path)
                })
                
        logger.info(f"스트림 목록에 {len([s for s in stream_list if s.get('type') == 'recording'])}개의 녹화영상 추가됨")
        
    except Exception as e:
        logger.error(f"녹화영상 목록 조회 중 오류: {e}")
    
    # 정렬: 웹캠 -> RTSP -> 녹화영상 순
    def sort_key(x):
        stream_type = x.get("type")
        if stream_type == "webcam":
            return 0
        elif stream_type == "rtsp":
            return 1
        elif stream_type == "recording":
            return 2
        else:
            return 3
    
    stream_list.sort(key=sort_key)
    
    return JSONResponse(content={"streams": stream_list})

@router.post("/refresh-streams")
async def refresh_streams(db: Session = Depends(get_db)):
    """사용 가능한 디바이스 새로고침 (실시간 스트림 + 녹화된 영상)"""
    # 스트림 목록 다시 초기화
    initialize_streams()
    
    # 실시간 스트림 목록을 리스트로 변환
    stream_list = []
    for stream_id, stream_data in streams.items():
        stream_list.append(stream_data)
    
    # 녹화된 영상들을 스트림 목록에 추가
    try:
        from app.models.video_stream import VideoStream
        from app.models.enums import StreamSourceType
        
        recordings = db.query(VideoStream).filter(
            VideoStream.source_type == StreamSourceType.WEBCAM
        ).order_by(VideoStream.recording_started_at.desc()).limit(20).all()  # 최근 20개만
        
        for recording in recordings:
            if recording.recording_path and os.path.exists(recording.recording_path):
                stream_list.append({
                    "id": f"recording_{recording.id}",
                    "type": "recording",
                    "path": recording.recording_path,
                    "url": recording.url,
                    "display_name": f"녹화영상 {recording.recording_started_at.strftime('%Y-%m-%d %H:%M')}",
                    "status": "recorded",
                    "client_count": 0,
                    "recording_started_at": recording.recording_started_at.isoformat() if recording.recording_started_at else None,
                    "recording_ended_at": recording.recording_ended_at.isoformat() if recording.recording_ended_at else None,
                    "file_size": os.path.getsize(recording.recording_path)
                })
                
        logger.info(f"새로고침된 스트림 목록에 {len([s for s in stream_list if s.get('type') == 'recording'])}개의 녹화영상 추가됨")
        
    except Exception as e:
        logger.error(f"새로고침 시 녹화영상 목록 조회 중 오류: {e}")
    
    # 정렬: 웹캠 -> RTSP -> 녹화영상 순
    def sort_key(x):
        stream_type = x.get("type")
        if stream_type == "webcam":
            return 0
        elif stream_type == "rtsp":
            return 1
        elif stream_type == "recording":
            return 2
        else:
            return 3
    
    stream_list.sort(key=sort_key)
    
    return JSONResponse(content={"streams": stream_list})

@router.post("/rtsp-stream")
async def add_rtsp_stream(rtsp_url: str, stream_id: str = None, display_name: str = None):
    """새 RTSP 스트림 추가"""
    # RTSP URL 유효성 검사
    if not rtsp_url.startswith("rtsp://"):
        return JSONResponse(
            status_code=400,
            content={"error": "유효하지 않은 RTSP URL 형식입니다."}
        )
    
    if not stream_id:
        stream_id = f"rtsp_{str(uuid.uuid4())[:8]}"
    
    # 표시 이름이 없는 경우 기본값 설정
    if not display_name:
        display_name = f"RTSP 스트림 {stream_id.split('_')[-1]}"
    
    # 스트림 등록
    streams[stream_id] = {
        "id": stream_id,
        "type": "rtsp",
        "url": rtsp_url,
        "display_name": display_name,
        "status": "ready",
        "client_count": 0
    }
    
    logging.info(f"RTSP 스트림 등록: {stream_id}, URL: {rtsp_url}")
    
    # 스트림 목록을 리스트로 변환
    stream_list = []
    for stream_id, stream_data in streams.items():
        stream_list.append(stream_data)
    
    # 웹캠 -> RTSP 순으로 정렬
    stream_list.sort(key=lambda x: 0 if x.get("type") == "webcam" else 1)
    
    return JSONResponse(content={"stream_id": stream_id, "status": "registered", "streams": stream_list})

@router.websocket("/signaling/{stream_id}")
async def websocket_endpoint(websocket: WebSocket, stream_id: str, db: Session = Depends(get_db)):
    """WebRTC 시그널링을 위한 웹소켓 엔드포인트"""
    try:
        # 연결 전 스트림 검증
        stream = find_by_id_in_dict(streams, stream_id)
        if not stream:
            logging.error(f"스트림 ID '{stream_id}'가 존재하지 않습니다.")
            await websocket.accept()
            await websocket.send_json({"type": "error", "message": f"스트림 ID '{stream_id}'가 존재하지 않습니다."})
            await websocket.close()
            return

        # 웹소켓 연결 수락
        await websocket.accept()
        logging.info(f"스트림 ID '{stream_id}'에 대한 새 WebSocket 연결 수락됨")

        # 세션 ID 생성 및 클라이언트 등록
        session_id = str(uuid.uuid4())
        client_id = f"{session_id}_{stream_id}"
        client_register[client_id] = websocket
        
        # 세션 생성 메시지 전송
        await websocket.send_json({
            "type": "session_created",
            "session_id": session_id
        })
        
        # 미디어 서버 연결 준비
        media_process = None
        
        try:
            # 스트림 타입에 따라 미디어 처리
            if stream.get("type") == "webcam":
                device_path = stream.get("path")
                if os.path.exists(device_path):
                    logging.info(f"웹캠 디바이스 '{device_path}' 연결 시작")
                    # 여기서 ffmpeg 또는 GStreamer로 미디어 서버 연결 설정
                    # 현재는 간단한 로깅만 추가
                    stream["status"] = "active"
                else:
                    logging.error(f"웹캠 디바이스 '{device_path}'가 존재하지 않습니다")
                    await websocket.send_json({
                        "type": "error", 
                        "message": f"웹캠 디바이스 '{device_path}'가 존재하지 않습니다"
                    })
            
            # 메시지 수신 대기
            last_message_time = time.time()
            
            while True:
                # 30초 타임아웃으로 메시지 대기
                try:
                    # 타임아웃 체크 (30초 동안 메시지가 없으면 핑 전송)
                    current_time = time.time()
                    if current_time - last_message_time > 30:
                        # 핑 메시지 전송
                        await websocket.send_json({"type": "pong"})
                        logging.debug(f"클라이언트 {client_id}에 핑 전송")
                        last_message_time = current_time
                    
                    # 1초 타임아웃으로 메시지 수신
                    try:
                        data = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
                        last_message_time = time.time()  # 메시지 수신 시간 갱신
                    except asyncio.TimeoutError:
                        # 타임아웃은 정상적인 상황, 다음 반복으로 진행
                        continue
                    
                    # JSON 파싱
                    try:
                        json_data = json.loads(data)
                    except json.JSONDecodeError as e:
                        logging.error(f"잘못된 JSON 형식: {e}")
                        continue
                    
                    # 메시지 타입에 따른 처리
                    msg_type = json_data.get("type")
                    
                    # 핑 처리
                    if msg_type == "ping":
                        await websocket.send_json({"type": "pong"})
                        continue
                    
                    # offer 처리
                    elif msg_type == "offer":
                        sdp = json_data.get("sdp")
                        if not sdp:
                            logging.warning("SDP가 누락된 offer")
                            continue
                        
                        logging.info(f"클라이언트 {client_id}로부터 offer 수신")
                        
                        # offer 수신 확인 메시지 전송
                        await websocket.send_json({"type": "offer_received"})
                        
                        # 세션 저장
                        webrtc_sessions[session_id] = {
                            "id": session_id,
                            "client_id": client_id,
                            "stream_id": stream_id,
                            "offer_sdp": sdp,
                            "created_at": time.time()
                        }
                        
                        # 서버 측 answer SDP 생성
                        # 현재는 간단한 더미 answer 생성 (실제로는 미디어 서버에서 생성)
                        # WebRTC SDP 형식에 맞게 응답 생성
                        answer_sdp = await generate_valid_answer_sdp(sdp, session_id)
                        
                        # SDP 응답 전송
                        await websocket.send_json({
                            "type": "answer",
                            "sdp": answer_sdp
                        })
                        
                        # 스트림 상태 업데이트
                        if stream_id in streams:
                            streams[stream_id]["status"] = "active"
                            
                        logging.info(f"SDP answer가 클라이언트 {client_id}에 전송됨")
                        
                    # 스트림 정보 요청 처리
                    elif msg_type == "get_stream_info":
                        stream = find_by_id_in_dict(streams, stream_id)
                        if stream:
                            logging.info(f"클라이언트 {client_id}에 스트림 정보 전송")
                            await websocket.send_json({
                                "type": "stream_info",
                                "stream": stream
                            })
                        else:
                            logging.error(f"요청된 스트림 ID '{stream_id}'를 찾을 수 없음")
                            await websocket.send_json({
                                "type": "error",
                                "message": f"스트림 ID '{stream_id}'를 찾을 수 없습니다."
                            })
                    
                    # ICE 후보 처리
                    elif msg_type == "ice_candidate":
                        candidate = json_data.get("candidate")
                        if not candidate:
                            logging.warning("후보 정보가 누락된 ice_candidate")
                            continue
                        
                        logging.debug(f"클라이언트 {client_id}로부터 ICE 후보 수신")
                        
                        # 실제로는 미디어 서버에 ICE 후보 전달 필요
                        # 현재는 로깅만 수행
                        
                        # 더미 ICE 후보 응답 (실제로는 미디어 서버에서 생성)
                        await asyncio.sleep(0.1)  # 잠시 대기
                        await websocket.send_json({
                            "type": "ice_candidate",
                            "candidate": candidate  # 실제로는 서버측 후보 정보
                        })
                    
                    else:
                        logging.warning(f"알 수 없는 메시지 타입: {msg_type}")
                
                except (WebSocketDisconnect, ConnectionClosedOK, ConnectionClosedError) as e:
                    logging.info(f"WebSocket 연결 종료: {e}")
                    break
                
                except Exception as e:
                    logging.error(f"메시지 처리 중 예외 발생: {str(e)}")
                    # 치명적이지 않은 오류는 연결 유지 시도
                    continue
        
        finally:
            # 미디어 프로세스 정리
            if media_process:
                try:
                    media_process.terminate()
                    media_process.wait(timeout=1)
                    logging.info(f"미디어 프로세스 정상 종료: {client_id}")
                except Exception as e:
                    logging.error(f"미디어 프로세스 종료 중 오류: {str(e)}")
                    try:
                        media_process.kill()
                    except:
                        pass
            
            # 스트림 상태 업데이트
            if stream_id in streams:
                streams[stream_id]["status"] = "ready"
                
            # WebRTC 세션 정리
            if session_id in webrtc_sessions:
                del webrtc_sessions[session_id]
    
    except Exception as e:
        logging.error(f"WebSocket 처리 중 예외 발생: {str(e)}")

        
    finally:
        # 클라이언트 등록 해제 및 정리
        try:
            if client_id in client_register:
                del client_register[client_id]
                logging.info(f"클라이언트 {client_id} 등록 해제됨")
        except Exception as e:
            logging.error(f"클라이언트 등록 해제 중 오류: {str(e)}")
        
        for webcam in DEFAULT_WEBCAMS:
            resource_manager.release_device(webcam, session_id)

        
        # 웹소켓 닫기 시도
        try:
            await websocket.close()
        except Exception:
            pass

@router.get("/session/{session_id}")
async def get_session(session_id: str):
    """세션 정보 조회 API"""
    session = rtc_manager.get_session(session_id)
    if not session:
        raise HTTPException(status_code=404, detail="세션을 찾을 수 없습니다")
    
    # 민감한 정보 제외
    safe_session = {
        "id": session["id"],
        "status": session["status"],
        "created_at": session["created_at"]
    }
    
    return JSONResponse(content=safe_session)

# 전역 MediaRelay 및 공유 비디오 트랙
media_relay = MediaRelay()
shared_video = None
logger = logging.getLogger("robodine.live_streaming")

import logging
import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
from aiortc.contrib.media import MediaPlayer

# 전역 리소스 및 공유 비디오 소스
logger = logging.getLogger("robodine.live_streaming")
media_relay = MediaRelay()
shared_video_source = None

async def generate_valid_answer_sdp(offer_sdp: str, session_id: str = None) -> str:
    """
    aiortc RTCPeerConnection을 사용하여 브라우저와 동일한 방식으로 SDP answer 자동 생성
    UDP 스트림을 이용한 비디오 트랙 제공
    """
    # 성능 측정 시작
    sdp_start_time = time.time()
    
    # 세션 ID 보장
    if not session_id:
        session_id = str(uuid.uuid4())
    logger.info(f"클라이언트 offer SDP 수신 (세션: {session_id})")

    # PeerConnection 생성
    config = RTCConfiguration(
        iceServers=[
            RTCIceServer(urls="stun:stun.l.google.com:19302"),
            RTCIceServer(urls="stun:stun1.l.google.com:19302")
        ]
    )
    pc = RTCPeerConnection(configuration=config)

    try:
        # 1) 오디오 트랙 추가 (무음)
        audio_start_time = time.time()
        try:
            audio_player = MediaPlayer(
                'anullsrc=sample_rate=48000:channel_layout=stereo',
                format='lavfi', options={'f': 'lavfi'}
            )
            if audio_player.audio:
                pc.addTrack(audio_player.audio)
                logger.info(f"더미 오디오 트랙 추가됨 (세션: {session_id})")
        except Exception as e:
            logger.warning(f"오디오 트랙 추가 실패 (세션: {session_id}): {e}")
        
        audio_end_time = time.time()
        performance_monitor.record_processing_time('audio_track_setup', audio_start_time, audio_end_time)

        # 2) 비디오 트랙 추가 - UDP 스트림 우선
        video_start_time = time.time()
        video_track_added = False

        # UDP 스트림에서 비디오 트랙 생성
        try:
            if udp_receiver.has_frames():
                # UDP 스트림이 활성화된 경우
                udp_video_track = UDPVideoStreamTrack(udp_receiver)
                pc.addTrack(udp_video_track)
                logger.info(f"UDP 비디오 트랙 추가됨 (세션: {session_id})")
                video_track_added = True
                performance_monitor.record_webrtc_metric('udp_video_track_used', 1, session_id)
            else:
                logger.warning(f"UDP 스트림에 프레임이 없음 (세션: {session_id})")
                performance_monitor.record_webrtc_metric('udp_video_track_unavailable', 1, session_id)
        except Exception as e:
            logger.warning(f"UDP 비디오 트랙 추가 실패 (세션: {session_id}): {e}")

        # 3) UDP 스트림 실패 시 더미 비디오 트랙 사용
        if not video_track_added:
            try:
                # 테스트 패턴 비디오 생성
                dummy_video = MediaPlayer(
                    'testsrc=size=640x480:rate=30', format='lavfi', options={'f': 'lavfi'}
                )
                if dummy_video.video:
                    pc.addTrack(dummy_video.video)
                    logger.info(f"더미 비디오 트랙 추가됨 (세션: {session_id})")
                    video_track_added = True
                    performance_monitor.record_webrtc_metric('dummy_video_track_used', 1, session_id)
            except Exception as ve:
                logger.error(f"더미 비디오 트랙 추가 실패: {ve}")

        # 4) 최종 폴백: 검은 화면 트랙
        if not video_track_added:
            try:
                black_video = MediaPlayer(
                    'color=black:size=640x480:rate=30', format='lavfi', options={'f': 'lavfi'}
                )
                if black_video.video:
                    pc.addTrack(black_video.video)
                    logger.info(f"검은 화면 비디오 트랙 추가됨 (세션: {session_id})")
                    performance_monitor.record_webrtc_metric('black_video_track_used', 1, session_id)
            except Exception as bve:
                logger.error(f"검은 화면 트랙 추가 실패: {bve}")
        
        video_end_time = time.time()
        performance_monitor.record_processing_time('video_track_setup', video_start_time, video_end_time)

        # 5) 원격 offer 설정 및 answer 생성
        offer_process_start = time.time()
        offer = RTCSessionDescription(sdp=offer_sdp, type="offer")
        await pc.setRemoteDescription(offer)
        logger.info(f"클라이언트 offer를 원격 설명으로 설정 완료 (세션: {session_id})")

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        offer_process_end = time.time()
        
        performance_monitor.record_processing_time('sdp_offer_answer', offer_process_start, offer_process_end)

        # 6) SDP 반환
        answer_sdp = pc.localDescription.sdp
        
        # 전체 SDP 처리 시간 측정
        sdp_end_time = time.time()
        total_sdp_time = (sdp_end_time - sdp_start_time) * 1000
        performance_monitor.record_processing_time('total_sdp_processing', sdp_start_time, sdp_end_time)
        
        logger.info(f"SDP answer 자동 생성 완료 (세션: {session_id}) - 총 처리시간: {total_sdp_time:.1f}ms")
        performance_monitor.record_webrtc_metric('sdp_processing_time_ms', total_sdp_time, session_id)
        
        return answer_sdp
        
    except Exception as e:
        sdp_error_time = time.time()
        error_processing_time = (sdp_error_time - sdp_start_time) * 1000
        logger.error(f"SDP 처리 중 오류 (세션: {session_id}) - 처리시간: {error_processing_time:.1f}ms, 오류: {e}")
        performance_monitor.record_webrtc_metric('sdp_processing_error', 1, session_id)
        raise e
    finally:
        # PC 리소스 정리는 클라이언트에서 관리하므로 여기서는 하지 않음
        pass

def generate_minimal_answer_sdp() -> str:
    """최소한의 기본 answer SDP (aiortc 실패시 대체용)"""
    return """v=0
o=- 1234567890 1 IN IP4 127.0.0.1
s=-
t=0 0
m=video 9 UDP/TLS/RTP/SAVPF 96
c=IN IP4 127.0.0.1
a=ice-ufrag:4ZcD
a=ice-pwd:2/1muCWoOi3uycBUlpKleupc
a=fingerprint:sha-256 E7:8A:84:72:33:A0:9E:28:6D:FB:BA:58:23:78:60:65:A2:C8:AA:3D:A1:73:92:DC:1C:74:27:15:24:78:77:B2
a=setup:active"""

@router.get("/streams/{stream_id}")
async def get_stream_info(stream_id: str):
    """특정 스트림 정보를 가져옵니다."""
    stream = find_by_id_in_dict(streams, stream_id)
    if not stream:
        return JSONResponse(
            status_code=404,
            content={"error": f"스트림 ID '{stream_id}'가 존재하지 않습니다."}
        )
    
    return JSONResponse(content={"stream": stream})

@router.post("/webrtc/ice-candidate")
async def handle_ice_candidate(request_data: dict):
    """ICE 후보를 처리하는 HTTP 엔드포인트"""
    try:
        session_id = request_data.get("sessionId")
        candidate = request_data.get("candidate")
        
        if not session_id or not candidate:
            return JSONResponse(
                status_code=400,
                content={"error": "sessionId와 candidate가 필요합니다."}
            )
        
        # 더미 응답
        return JSONResponse(content={
            "type": "iceCandidate",
            "accepted": True
        })
        
    except Exception as e:
        logger.error(f"ICE 후보 처리 중 오류: {str(e)}")
        return JSONResponse(
            status_code=500,
            content={"error": "서버 내부 오류", "details": str(e)}
        )

# REST API 엔드포인트 추가 - WebSocket 대신 사용
@router.post("/webrtc/offer")
async def handle_webrtc_offer(request_data: dict):
    """SDP offer를 받고 answer를 반환하는 HTTP 엔드포인트 (aiortc 사용, 개선된 리소스 관리)"""
    try:
        stream_id = request_data.get("streamId")
        sdp_offer = request_data.get("sdp")
        client_ip = request_data.get("clientIp", "unknown")
        
        if not stream_id or not sdp_offer:
            return JSONResponse(
                status_code=400, 
                content={"error": "streamId와 sdp가 필요합니다."}
            )
        
        # 클라이언트 키 생성 (IP + stream_id)
        client_key = f"{client_ip}_{stream_id}"
        
        # 만료된 세션 정리
        resource_manager.cleanup_expired_sessions()
        
        # 완화된 Rate limiting 체크 (15초 내 5회 허용)
        if resource_manager.is_rate_limited(client_key, max_requests=5, time_window=15):
            return JSONResponse(
                status_code=429,
                content={
                    "error": "요청이 너무 빈번합니다. 잠시 후 다시 시도하세요.",
                    "retry_after": 15
                }
            )
        
        # 기존 활성 세션 수 확인
        active_sessions = resource_manager.get_client_session_count(client_key)
        if active_sessions > 2:  # 클라이언트당 최대 3개 세션 허용
            logger.warning(f"클라이언트 {client_key}의 활성 세션이 너무 많음: {active_sessions}")
            # 기존 세션들 정리
            webcam_device = '/dev/video2'
            for device_path, session_id in list(resource_manager._device_users.items()):
                if client_key in session_id:
                    resource_manager.release_device(device_path, session_id)
                    logger.info(f"기존 세션 정리됨: {session_id}")
        
        # 스트림 존재 확인
        stream = find_by_id_in_dict(streams, stream_id)
        if not stream:
            return JSONResponse(
                status_code=404,
                content={"error": f"스트림 ID '{stream_id}'가 존재하지 않습니다."}
            )
        
        # 세션 ID 생성 (클라이언트 키와 타임스탬프 포함)
        timestamp = int(time.time())
        session_id = f"{client_key}_{timestamp}_{str(uuid.uuid4())[:8]}"
        
        logger.info(f"WebRTC offer 요청 처리 시작 - 스트림: {stream_id}, 세션: {session_id}, 클라이언트: {client_key}")
        
        # aiortc를 사용하여 브라우저 내장 API와 동일한 방식으로 SDP answer 생성
        answer_sdp = await generate_valid_answer_sdp(sdp_offer, session_id)

        logger.info(f"스트림 ID '{stream_id}'에 대한 SDP offer 처리 완료")
        logger.info(f"aiortc로 생성된 SDP answer 전송 (세션: {session_id})")
        
        # 스트림 상태 업데이트
        if stream_id in streams:
            streams[stream_id]["status"] = "active"
        
        # ICE 후보 생성 (더미)
        ice_candidates = [
            {
                "candidate": "candidate:1234567890 1 udp 2122262783 127.0.0.1 30000 typ host generation 0",
                "sdpMid": "video",
                "sdpMLineIndex": 1
            }
        ]
        
        return JSONResponse(content={
            "type": "answer",
            "sdp": answer_sdp,
            "sessionId": session_id,
            "iceCandidates": ice_candidates
        })
        
    except Exception as e:
        logger.error(f"WebRTC offer 처리 중 오류: {str(e)}")
        return JSONResponse(
            status_code=500,
            content={"error": "서버 내부 오류", "details": str(e)}
        )

@router.delete("/webrtc/session/{session_id}")
async def close_webrtc_session(session_id: str):
    """WebRTC 세션 종료 및 리소스 정리"""
    try:
        # 디바이스 리소스 해제
        webcam_device = '/dev/video2'
        resource_manager.release_device(webcam_device, session_id)
        
        # 세션 정리
        if session_id in webrtc_sessions:
            del webrtc_sessions[session_id]
        
        logger.info(f"WebRTC 세션 정리 완료: {session_id}")

        for webcam in DEFAULT_WEBCAMS:
            resource_manager.release_device(webcam, session_id)
        
        return JSONResponse(content={
            "status": "closed",
            "sessionId": session_id
        })
        
    except Exception as e:
        logger.error(f"세션 정리 중 오류: {str(e)}")
        return JSONResponse(
            status_code=500,
            content={"error": "세션 정리 실패", "details": str(e)}
        )

@router.get("/device-status")
async def get_device_status():
    """현재 디바이스 사용 상태 조회"""
    try:
        status = {
            "devices": resource_manager._device_users.copy(),
            "active_sessions": len(resource_manager._device_users),
            "webcam_available": "/dev/video2" not in resource_manager._device_users
        }
        
        return JSONResponse(content=status)
        
    except Exception as e:
        logger.error(f"디바이스 상태 조회 중 오류: {str(e)}")
        return JSONResponse(
            status_code=500,
            content={"error": "디바이스 상태 조회 실패", "details": str(e)}
        )

# 클라이언트 성능 메트릭 수신 엔드포인트 추가
@router.post("/client-metrics")
async def receive_client_metrics(request_data: dict):
    """클라이언트에서 전송된 성능 메트릭 수신 및 로깅"""
    try:
        metric_type = request_data.get("type")
        metrics = request_data.get("metrics", {})
        
        if metric_type == "webrtc_client_performance":
            logger.info("=== 클라이언트 WebRTC 성능 메트릭 ===")
            logger.info(f"연결 시작 → Offer 생성: {metrics.get('connectionStartToOffer', 0):.1f}ms")
            logger.info(f"Offer → Answer 수신: {metrics.get('offerToAnswer', 0):.1f}ms")
            logger.info(f"Answer → 첫 프레임: {metrics.get('answerToFirstFrame', 0):.1f}ms")
            logger.info(f"총 연결 시간: {metrics.get('totalConnectionTime', 0):.1f}ms")
            logger.info(f"클라이언트 User-Agent: {metrics.get('clientUserAgent', 'Unknown')}")
            logger.info(f"메트릭 수신 시간: {metrics.get('timestamp', 'Unknown')}")
            
            # 전역 성능 모니터에 클라이언트 메트릭 기록
            performance_monitor.record_webrtc_metric('client_total_connection_time_ms', metrics.get('totalConnectionTime', 0))
            performance_monitor.record_webrtc_metric('client_offer_to_answer_ms', metrics.get('offerToAnswer', 0))
            
            # 긴 연결 시간 경고
            total_time = metrics.get('totalConnectionTime', 0)
            if total_time > 3000:  # 3초 이상
                logger.warning(f"⚠️ 긴 클라이언트 연결 시간 감지: {total_time:.1f}ms")
            elif total_time > 5000:  # 5초 이상
                logger.error(f"❌ 매우 긴 클라이언트 연결 시간: {total_time:.1f}ms")
                
        else:
            logger.info(f"기타 클라이언트 메트릭 수신: {metric_type}")
            logger.debug(f"메트릭 데이터: {metrics}")
        
        return JSONResponse(content={"status": "received", "type": metric_type})
        
    except Exception as e:
        logger.error(f"클라이언트 메트릭 처리 중 오류: {str(e)}")
        return JSONResponse(
            status_code=500,
            content={"error": "메트릭 처리 실패", "details": str(e)}
        )

# 현재 스트리밍 성능 상태 조회 엔드포인트
@router.get("/performance-status")
async def get_performance_status():
    """현재 스트리밍 성능 상태 조회"""
    try:
        fps = performance_monitor.calculate_fps()
        avg_latency = performance_monitor.calculate_average_latency()
        
        # 최근 메트릭들 수집
        recent_processing = list(performance_monitor.processing_times)[-10:] if performance_monitor.processing_times else []
        recent_webrtc = list(performance_monitor.webrtc_metrics)[-10:] if performance_monitor.webrtc_metrics else []
        
        status = {
            "current_fps": round(fps, 2),
            "average_processing_latency_ms": round(avg_latency, 2),
            "total_frames_processed": performance_monitor.frame_count,
            "udp_receiver_active": udp_receiver.has_frames(),
            "recent_processing_times": [
                {
                    "process": p['process'],
                    "time_ms": round(p['time_ms'], 2),
                    "timestamp": p['timestamp']
                } for p in recent_processing
            ],
            "recent_webrtc_metrics": [
                {
                    "metric": m['metric'],
                    "value": m['value'],
                    "timestamp": m['timestamp']
                } for m in recent_webrtc
            ]
        }
        
        return JSONResponse(content=status)
        
    except Exception as e:
        logger.error(f"성능 상태 조회 중 오류: {str(e)}")
        return JSONResponse(
            status_code=500,
            content={"error": "성능 상태 조회 실패", "details": str(e)}
        )