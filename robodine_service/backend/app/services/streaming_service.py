# robodine_service/backend/app/services/streaming_service.py
import os
import time
import threading
import logging
from datetime import datetime
import cv2
import subprocess

logger = logging.getLogger("robodine.streaming_service")

# 전역 스트림 URL 리스트
_stream_urls: list[str] = []
_active_streams = {}  # 활성 스트림 정보 저장
_average_latencies = {}  # 평균 레이턴시 저장

# 스트림 URL 관리
def get_stream_urls() -> list[str]:
    return _stream_urls

# 스트림 URL을 추가하고 쓰레드를 시작
def add_stream_url(url: str, path: str = None):
    if url not in _stream_urls:
        _stream_urls.append(url)

        # _active_streams에 해당 URL에 대한 정보 저장
        stop_event = threading.Event()
        latencies = []
        _active_streams[url] = {"stop_event": stop_event, "latencies": latencies}

        # 별도의 쓰레드에서 record_stream 실행
        threading.Thread(target=record_stream, args=(url, stop_event, latencies, path), daemon=True).start()

# 스트리밍 서비스에서 URL 제거
def remove_stream_url(url: str):
    _stream_urls.remove(url)


# 파일 경로 생성
def get_file_path(rtsp_url: str, path: str = None) -> str:
    ip = rtsp_url.split("//")[1].split(":")[0]
    base_dir = os.path.join("videos", ip)
    os.makedirs(base_dir, exist_ok=True)
    # path를 이용한 파일 이름 생성
    file_name = os.path.basename(path)
    return os.path.join(base_dir, file_name)


# 스트리밍을 처리하는 함수
def record_stream(rtsp_url: str, stop_event: threading.Event, latencies: list[float], path: str = None):
        ip = rtsp_url.split("//")[1].split(":")[0]
        cap = cv2.VideoCapture(rtsp_url)
        if not cap.isOpened():
            logger.warning(f"[RTSP:{ip}] Connection failed, retrying in 5s...")
            time.sleep(5)
            return

        width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps    = cap.get(cv2.CAP_PROP_FPS) or 20.0
        out_path = get_file_path(rtsp_url, path)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(out_path, fourcc, fps, (width, height))
        logger.info(f"[RTSP:{ip}] Recording to {out_path}")

        while not stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                logger.warning(f"[RTSP:{ip}] Stream ended")
                break
            writer.write(frame)

        cap.release()
        writer.release()

        # --- H.264 변환 작업 시작 ---
        # 변환 전용 임시 파일 경로
        tmp_path = out_path + ".tmp.mp4"
        # FFmpeg로 H.264 + AAC 코덱으로 트랜스코딩
        try:
            subprocess.run([
                "ffmpeg",
                "-y",  # overwrite without asking
                "-i", out_path,
                "-c:v", "libx264",
                "-preset", "fast",
                "-crf", "23",
                "-c:a", "aac",
                "-movflags", "+faststart",
                tmp_path
            ], check=True)
            # 원본 파일을 백업하고, 변환된 파일을 원래 이름으로 교체
            os.replace(tmp_path, out_path)
            logger.info(f"[RTSP:{ip}] Transcoded to H.264: {out_path}")
        except Exception as e:
            logger.error(f"[RTSP:{ip}] H.264 변환 실패: {e}")

        # 평균 레이턴시 계산
        avg_latency = sum(latencies) / len(latencies) if latencies else 0.0
        _average_latencies[rtsp_url] = avg_latency
        logger.info(f"[RTSP:{ip}] Finished {out_path} with avg latency: {avg_latency:.3f}s")
        time.sleep(1)
        
        # 여기서 함수 종료 — 연결 끊김
        logger.info(f"[RTSP:{ip}] Recording session completed, closing connection.")
        return