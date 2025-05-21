# robodine_service/backend/app/routes/live_streaming.py
import asyncio
import logging
import os

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()
logger = logging.getLogger("robodine.live_streaming")
logger.setLevel(logging.DEBUG)

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

# 현재 진행 중인 ffmpeg 프로세스 참조
current_proc = None

async def check_webcam_available(device_path):
    """웹캠 장치 사용 가능 여부 확인"""
    if not os.path.exists(device_path):
        return False
    try:
        process = await asyncio.create_subprocess_exec(
            "ffmpeg", "-loglevel", "error", "-f", "v4l2", "-list_formats", "all", "-i", device_path,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        _, stderr = await process.communicate()
        return process.returncode == 0 or b"Input/output error" not in stderr
    except Exception as e:
        logger.error(f"웹캠 확인 중 오류: {e}")
        return False

@router.websocket("/webcam")
async def websocket_webcam(websocket: WebSocket):
    global current_proc
    await websocket.accept()
    logger.info("웹캠 스트리밍 요청 수신")

    # 이미 스트리밍 중인 ffmpeg 프로세스가 있으면 거부
    if current_proc and current_proc.returncode is None:
        logger.warning("기존 스트림 진행 중, 신규 연결 거부")
        await websocket.close(code=1013, reason="Stream busy")
        return

    device = "/dev/video5"
    if not await check_webcam_available(device):
        logger.error(f"웹캠을 사용할 수 없습니다: {device}")
        await websocket.close(code=1008, reason="Webcam not available")
        return

    # ffmpeg 커맨드 구성
    cmd = [
        "ffmpeg", 
        "-loglevel", "debug", # 디버그 레벨 로깅
        "-f", "v4l2", # v4l2 포맷 사용
        "-input_format", "mjpeg", # MJPEG 포맷 사용
        "-fflags", "+nobuffer", # 버퍼링 비활성화
        "-flags", "low_delay", # 저지연 플래그 설정
        "-i", device, # 웹캠 장치 경로
        "-s", "640x480", # 크기 명시적 지정
        "-r", "15", # 프레임레이트 제한
        "-pix_fmt", "yuv420p", # 픽셀 포맷 설정
        "-f", "mpegts", # 출력 포맷 설정
        "-codec:v", "mpeg1video", # 비디오 코덱 설정
        "-b:v", "800k", # 비트레이트 설정
        "-q:v", "31", # 품질 설정 (0-31, 낮을수록 품질 높음)
        "-strict", "unofficial", # 비공식 코덱 사용 허용
        "-g", "15", # GOP 크기 설정 (15프레임)
        "-bf", "0", # B-프레임 없음
        "-an", # 오디오 제외
        "-"
    ]

    try:
        logger.info(f"ffmpeg 명령어: {' '.join(cmd)}")
        # ffmpeg 프로세스 시작
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        current_proc = proc
        logger.info("웹캠 ffmpeg 프로세스 시작됨")
        await asyncio.sleep(0.5)

        # 초기 데이터 청크 처리
        try:
            logger.info("초기 데이터 청크 읽기 시작...")
            initial_chunk = await asyncio.wait_for(proc.stdout.read(1024*1024), timeout=10.0)
            if not initial_chunk:
                logger.error("초기 데이터 없음, 연결 종료")
                await websocket.close(code=1011, reason="No initial data")
                return
            # TS 동기화 바이트 찾아서 전송
            offset = next((i for i, b in enumerate(initial_chunk[:100]) if b == 0x47), 0)
            initial_chunk = initial_chunk[offset:]
            await websocket.send_bytes(initial_chunk)
        except asyncio.TimeoutError:
            logger.error("초기 데이터 수신 타임아웃, 연결 종료")
            await websocket.close(code=1011, reason="Initial data timeout")
            return

        # 지속 스트리밍
        logger.info("지속적인 데이터 스트리밍 시작")
        chunk_size = 32 * 1024
        last_keepalive = asyncio.get_event_loop().time()

        while True:
            # Keep-alive: 2초마다 TS sync 패킷 전송
            now = asyncio.get_event_loop().time()
            if now - last_keepalive > 1.0:
                await websocket.send_bytes(b"\x47" * 188)
                last_keepalive = now

            try:
                chunk = await asyncio.wait_for(proc.stdout.read(chunk_size), timeout=0.5)
                if chunk:
                    await websocket.send_bytes(chunk)
                else:
                    # 빈 청크: 잠시 대기
                    await asyncio.sleep(0.1)
            except asyncio.TimeoutError:
                continue
            except WebSocketDisconnect:
                logger.info("클라이언트 연결 종료")
                break
            except Exception as e:
                logger.error(f"데이터 전송 중 오류: {e}")
                break

    except Exception as e:
        logger.error(f"ffmpeg 프로세스 시작 오류: {e}")
    finally:
        # ffmpeg 프로세스 종료 및 클린업
        if current_proc:
            proc_to_kill = current_proc
            current_proc = None
            try:
                proc_to_kill.kill()
                await proc_to_kill.wait()
            except Exception as e:
                logger.error(f"프로세스 정리 중 오류: {e}")
        logger.info("웹캠 ffmpeg 프로세스 종료됨")





@router.websocket("/video")
async def websocket_video(websocket: WebSocket, url: str):
    await websocket.accept()

    if not url:
        await websocket.close(code=1008, reason="URL parameter is required")
        return

    logger.info(f"RTSP 스트리밍 시작: {url}")

    # RTSP → mpegts (JSMpeg Canvas2D 호환 설정)
    cmd = [
        "ffmpeg",
        "-loglevel", "error",
        "-rtsp_transport", "tcp",
        "-i", url,
        
        # 출력 인코딩 설정 - Canvas2D 렌더러를 위해 최적화
        "-f", "mpegts",
        "-codec:v", "mpeg1video",
        "-b:v", "800k",
        "-q:v", "10",
        "-s", "640x480",      # 크기 명시적 지정
        "-r", "15",           # 프레임레이트 제한
        
        # 최적화 옵션
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-bf", "0",           # B-프레임 없음
        "-flags", "low_delay",
        "-fflags", "+nobuffer+flush_packets",
        "-an",                # 오디오 제외
        
        "-"
    ]

    proc = await asyncio.create_subprocess_exec(
        *cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )
    logger.info("RTSP ffmpeg 프로세스 시작됨")

    try:
        chunk_size = 4096
        while True:
            try:
                chunk = await asyncio.wait_for(proc.stdout.read(chunk_size), timeout=0.5)
                if not chunk:
                    logger.warning("RTSP 스트림 EOF")
                    break
                await websocket.send_bytes(chunk)
            except asyncio.TimeoutError:
                continue
            except WebSocketDisconnect:
                logger.info("클라이언트 연결 종료")
                break
            except Exception as e:
                logger.error(f"데이터 전송 중 오류: {e}")
                break
                
    finally:
        if 'proc' in locals():
            try:
                err = await proc.stderr.read()
                if err:
                    logger.error("ffmpeg(stderr):\n" + err.decode(errors="ignore"))
                
                proc.kill()
                await proc.wait()
            except Exception as e:
                logger.error(f"프로세스 정리 중 오류: {e}")
                
        logger.info("RTSP ffmpeg 프로세스 종료됨")