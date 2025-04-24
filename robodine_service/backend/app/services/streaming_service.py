# robodine_service/backend/app/services/streaming_service.py

import os
import time
import threading
import logging
from datetime import datetime
import cv2

logger = logging.getLogger("robodine.streaming_service")

# 전역 스트림 URL 리스트
_stream_urls: list[str] = []

def get_stream_urls() -> list[str]:
    return _stream_urls

def add_stream_url(url: str):
    if url not in _stream_urls:
        _stream_urls.append(url)
        threading.Thread(target=record_stream, args=(url,), daemon=True).start()

def remove_stream_url(url: str):
    _stream_urls.remove(url)

def record_stream(rtsp_url: str):
    ip = rtsp_url.split("//")[1].split(":")[0]
    base_dir = os.path.join("videos", ip)
    os.makedirs(base_dir, exist_ok=True)

    while rtsp_url in _stream_urls:
        cap = cv2.VideoCapture(rtsp_url)
        if not cap.isOpened():
            logger.warning(f"[RTSP:{ip}] Connection failed, retry in 5s...")
            time.sleep(5)
            continue

        width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps    = cap.get(cv2.CAP_PROP_FPS) or 20.0
        ts     = datetime.now().strftime("%Y%m%d%H%M%S")
        out_path = os.path.join(base_dir, f"stream_{ts}.mp4")

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(out_path, fourcc, fps, (width, height))
        logger.info(f"[RTSP:{ip}] Recording to {out_path}")

        while rtsp_url in _stream_urls:
            ret, frame = cap.read()
            if not ret:
                logger.warning(f"[RTSP:{ip}] Stream ended")
                break
            writer.write(frame)

        cap.release()
        writer.release()
        logger.info(f"[RTSP:{ip}] Finished {out_path}")
        time.sleep(1)
