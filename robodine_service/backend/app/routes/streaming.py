# robodine_service/backend/app/routes/streaming.py

import os
import time
import threading
import logging
from datetime import datetime
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import cv2

logger = logging.getLogger("robodine.streaming_service")
router = APIRouter()

class StreamRequest(BaseModel):
    url: str

# { url: {"stop_event":Event, "thread":Thread, "latencies":List[float]} }
_active_streams: dict[str, dict] = {}
_average_latencies: dict[str, float] = {}

def record_stream(rtsp_url: str, stop_event: threading.Event, latencies: list[float]):
    ip = rtsp_url.split("//")[1].split(":")[0]
    base_dir = os.path.join("videos", ip)
    os.makedirs(base_dir, exist_ok=True)

    origin = None
    while not stop_event.is_set():
        cap = cv2.VideoCapture(rtsp_url)
        if not cap.isOpened():
            logger.warning(f"[RTSP:{ip}] Connection failed, retrying in 5s...")
            time.sleep(5)
            continue

        width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps    = cap.get(cv2.CAP_PROP_FPS) or 20.0

        ts       = datetime.now().strftime("%Y%m%d%H%M%S")
        out_path = os.path.join(base_dir, f"stream_{ts}.mp4")
        fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
        writer   = cv2.VideoWriter(out_path, fourcc, fps, (width, height))

        logger.info(f"[RTSP:{ip}] Recording to {out_path}")

        # 프레임 루프
        while not stop_event.is_set():
            ret, frame = cap.read()
            rec_ts = time.time()
            if not ret:
                logger.warning(f"[RTSP:{ip}] Frame read failed, restarting capture")
                break

            # 스트림 상 상대적 타임스탬프(ms)
            pos_sec = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            if origin is None:
                # 첫 프레임 기준 원점 계산
                origin = rec_ts - pos_sec
            # 프레임 생성 시각 추정
            orig_ts = origin + pos_sec
            latencies.append(rec_ts - orig_ts)

            writer.write(frame)

        cap.release()
        writer.release()
        logger.info(f"[RTSP:{ip}] Finished {out_path}")
        time.sleep(1)

    # 평균 레이턴시 계산
    avg = sum(latencies) / len(latencies) if latencies else 0.0
    _average_latencies[rtsp_url] = avg
    logger.info(f"[RTSP:{ip}] Average latency: {avg:.3f}s")

@router.post("/add")
def add_stream(req: StreamRequest):
    url = req.url
    if url in _active_streams:
        return {"added": url}

    stop_event = threading.Event()
    latencies = []
    t = threading.Thread(
        target=record_stream,
        args=(url, stop_event, latencies),
        daemon=True
    )
    _active_streams[url] = {"stop_event": stop_event, "thread": t}
    t.start()
    logger.info(f"[STREAM] Started recording thread for {url}")
    return {"added": url}

@router.post("/remove")
def remove_stream(req: StreamRequest):
    url = req.url
    info = _active_streams.pop(url, None)
    if not info:
        # 이미 중지된 URL 도 200으로 처리
        return {"removed": url, "average_latency": None}

    # 중지 시그널
    info["stop_event"].set()
    # 최대 5초 대기
    info["thread"].join(timeout=5)

    avg = _average_latencies.pop(url, None)
    return {"removed": url, "average_latency": avg}
