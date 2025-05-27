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
from collections import defaultdict

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException, Depends
from starlette.responses import JSONResponse
from sqlalchemy.orm import Session

# aiortc 라이브러리 import 추가
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
from aiortc.contrib.media import MediaPlayer, MediaRelay

from app.core.db_config import get_db

router = APIRouter()
logger = logging.getLogger("robodine.live_streaming")
logger.setLevel(logging.DEBUG)

# 전역 리소스 관리
class DeviceResourceManager:
    def __init__(self):
        self._device_locks = {}  # 디바이스별 락
        self._device_users = {}  # 디바이스 사용자 추적
        self._session_timestamps = defaultdict(list)  # 세션별 요청 타임스탬프
        self._global_lock = threading.Lock()
        
    def acquire_device(self, device_path: str, session_id: str) -> bool:
        """디바이스 사용 권한 획득"""
        with self._global_lock:
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

# 고정 웹캠 설정
DEFAULT_WEBCAMS = [
    {"id": "camera_2", "path": "/dev/video2", "display_name": "메인 카메라"},
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
    
    # 웹캠 등록
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
        
        # 디바이스 존재 여부 로깅
        if os.path.exists(webcam["path"]):
            logging.info(f"웹캠 등록: {webcam_id}, 경로: {webcam['path']}")
        else:
            logging.warning(f"웹캠 강제 등록 (디바이스 없음): {webcam_id}, 경로: {webcam['path']}")
    
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
async def list_streams():
    """사용 가능한 모든 스트림 목록 반환"""
    # 스트림 목록을 리스트로 변환
    stream_list = []
    for stream_id, stream_data in streams.items():
        stream_list.append(stream_data)
    
    # 웹캠 -> RTSP 순으로 정렬
    stream_list.sort(key=lambda x: 0 if x.get("type") == "webcam" else 1)
    
    return JSONResponse(content={"streams": stream_list})

@router.post("/refresh-streams")
async def refresh_streams():
    """사용 가능한 디바이스 새로고침"""
    # 스트림 목록 다시 초기화
    initialize_streams()
    
    # 정렬된 스트림 목록 반환
    stream_list = []
    for stream_id, stream_data in streams.items():
        stream_list.append(stream_data)
    
    # 웹캠 -> RTSP 순으로 정렬
    stream_list.sort(key=lambda x: 0 if x.get("type") == "webcam" else 1)
    
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

async def generate_valid_answer_sdp(offer_sdp: str, session_id: str = None) -> str:
    """aiortc RTCPeerConnection을 사용하여 브라우저와 동일한 방식으로 SDP answer 자동 생성 (리소스 관리 포함)"""
    
    if not session_id:
        session_id = str(uuid.uuid4())
    
    logger.info(f"클라이언트 offer SDP 수신 (세션: {session_id}):\n{offer_sdp}")
    
    try:
        # RTCPeerConnection 생성 (aiortc) - RTCIceServer 객체 사용
        config = RTCConfiguration(
            iceServers=[
                RTCIceServer(urls="stun:stun.l.google.com:19302"),
                RTCIceServer(urls="stun:stun1.l.google.com:19302")
            ]
        )
        pc = RTCPeerConnection(configuration=config)
        
        # 더미 미디어 트랙 추가 (실제 웹캠 스트림을 시뮬레이션)
        # 오디오 트랙 추가
        try:
            # 더미 오디오 소스 생성 (사일런스)
            audio_player = MediaPlayer('anullsrc=sample_rate=48000:channel_layout=stereo', format='lavfi', options={
                'f': 'lavfi'
            })
            if audio_player.audio:
                pc.addTrack(audio_player.audio)
                logger.info(f"더미 오디오 트랙 추가됨 (세션: {session_id})")
        except Exception as e:
            logger.warning(f"오디오 트랙 추가 실패 (세션: {session_id}): {e}")
        
        # 비디오 트랙 추가 - 디바이스 경합 방지
        video_track_added = False
        webcam_device = '/dev/video2'
        
        # 웹캠 디바이스 사용 권한 확인
        if resource_manager.acquire_device(webcam_device, session_id):
            try:
                # 실제 웹캠 시도
                video_player = MediaPlayer(webcam_device, format='v4l2', options={
                    'video_size': '640x480',
                    'framerate': '30'
                })
                if video_player.video:
                    pc.addTrack(video_player.video)
                    logger.info(f"웹캠 비디오 트랙 추가됨 (세션: {session_id})")
                    video_track_added = True
            except Exception as e:
                logger.warning(f"웹캠 비디오 트랙 추가 실패 (세션: {session_id}): {e}")
                # 실패시 디바이스 해제
                resource_manager.release_device(webcam_device, session_id)
        else:
            logger.info(f"웹캠 디바이스 사용 중, 더미 트랙 사용 (세션: {session_id})")
        
        # 웹캠 실패시 반드시 더미 비디오 패턴 사용
        if not video_track_added:
            try:
                dummy_video_player = MediaPlayer('testsrc=size=640x480:rate=30', format='lavfi', options={
                    'f': 'lavfi'
                })
                if dummy_video_player.video:
                    pc.addTrack(dummy_video_player.video)
                    logger.info(f"더미 비디오 트랙 추가됨 (세션: {session_id})")
                    video_track_added = True
            except Exception as ve:
                logger.error(f"더미 비디오 트랙 추가도 실패 (세션: {session_id}): {ve}")
        
        # 어떤 트랙도 추가되지 않았다면 최소한의 검은 화면 트랙 추가
        if not video_track_added:
            try:
                black_video_player = MediaPlayer('color=black:size=640x480:rate=30', format='lavfi', options={
                    'f': 'lavfi'
                })
                if black_video_player.video:
                    pc.addTrack(black_video_player.video)
                    logger.info(f"검은 화면 비디오 트랙 추가됨 (세션: {session_id})")
                    video_track_added = True
            except Exception as bve:
                logger.error(f"검은 화면 비디오 트랙 추가도 실패 (세션: {session_id}): {bve}")
        
        if not video_track_added:
            logger.error(f"어떤 비디오 트랙도 추가할 수 없음 (세션: {session_id}) - SDP 생성에 영향을 줄 수 있음")
        
        # 클라이언트 offer를 원격 설명으로 설정
        offer = RTCSessionDescription(sdp=offer_sdp, type="offer")
        await pc.setRemoteDescription(offer)
        logger.info(f"클라이언트 offer를 원격 설명으로 설정 완료 (세션: {session_id})")
        
        # 브라우저 내장 API와 동일한 방식으로 answer 자동 생성
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        
        # 생성된 answer SDP 추출
        answer_sdp = pc.localDescription.sdp
        
        logger.info(f"브라우저 내장 API 방식으로 SDP answer 자동 생성 완료 (세션: {session_id})")
        logger.debug(f"생성된 answer SDP (세션: {session_id}):\n{answer_sdp}")
        
        # TODO: pc.close()는 실제 스트리밍이 끝날 때 호출해야 함
        # 현재는 리소스 관리를 위해 주석 처리
        
        return answer_sdp
        
    except Exception as e:
        logger.error(f"aiortc SDP answer 생성 중 오류 (세션: {session_id}): {str(e)}")
        # 오류 발생시 디바이스 해제
        resource_manager.release_device(webcam_device, session_id)
        # 오류 발생시 기본 응답 반환
        return generate_minimal_answer_sdp()

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