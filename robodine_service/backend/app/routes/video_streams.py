from fastapi import APIRouter, Depends, HTTPException, status, Response
from sqlalchemy.orm import Session
from typing import List, Optional, Dict, Any
from datetime import datetime
import os
from pydantic import BaseModel, Field
import glob
import time
import threading
import logging
import cv2

from app.core.db_config import get_db
from app.models import VideoStream, User
from app.models.enums import StreamSourceType, StreamStatus, UserRole
from app.routes.auth import get_current_user
from app.services.streaming_service import add_stream_url, remove_stream_url, get_file_path

router = APIRouter()

# --- Video Stream Models ---
class VideoStreamResponse(BaseModel):
    id: int
    source_type: StreamSourceType
    source_id: str
    recording_path: str
    last_checked: datetime
    status: StreamStatus
    url: str
    recording_started_at: Optional[datetime] = None
    recording_ended_at: Optional[datetime] = None

class VideoStreamRequest(BaseModel):
    source_type: StreamSourceType
    source_id: Optional[str] = None
    recording_path: str
    url: str
    recording_started_at: datetime
    recording_ended_at: datetime

# --- Router Endpoints ---
@router.post("/register", response_model=VideoStreamResponse)
def add_video_stream(
    stream_data: VideoStreamRequest,
    db: Session = Depends(get_db)):

    # 새 스트림 생성
    new_stream = VideoStream(
        source_type=stream_data.source_type,
        source_id=stream_data.source_id,
        url=stream_data.url,
        recording_path=stream_data.recording_path,
        recording_started_at=stream_data.recording_started_at,
        recording_ended_at=stream_data.recording_ended_at,
        last_checked=datetime.utcnow(),
        status=StreamStatus.ACTIVE
    )
    
    db.add(new_stream)
    db.commit()
    
    # 스트리밍 서비스에 URL 추가
    add_stream_url(stream_data.url)
    
    return {"status": "success", "message": "스트림이 추가되었습니다."}

@router.post("/remove", response_model=dict)
def remove_video_stream(
    stream_data: VideoStreamResponse,
    db: Session = Depends(get_db)
):
    # 스트림 찾기
    stream = db.query(VideoStream).filter(
        VideoStream.url == stream_data.url
    ).first()
    
    if stream:
        # 상태 업데이트
        stream.status = StreamStatus.INACTIVE
        stream.last_checked = datetime.utcnow()
        db.add(stream)
        db.commit()
    
    # # 스트리밍 서비스에서 URL 제거
    # remove_stream_url(stream_data.url)
    
    return {"status": "success", "message": "스트림이 비활성화되었습니다."}

@router.get("", response_model=List[VideoStreamResponse])
def get_video_streams(db: Session = Depends(get_db)):
    streams = db.query(VideoStream).all()
    result = [
        VideoStreamResponse(
            id=stream.id,
            source_type=stream.source_type,
            source_id=stream.source_id,
            recording_path=stream.recording_path,
            last_checked=stream.last_checked,
            status=stream.status,
            url=stream.url,
            recording_started_at=stream.recording_started_at,
            recording_ended_at=stream.recording_ended_at
        ) for stream in streams
    ]
    return result

@router.get("/recordings/{stream_id}", response_model=VideoStreamResponse)
def get_stream_recordings(
    stream_id: int,
    db: Session = Depends(get_db)
):
    stream = db.query(VideoStream).filter(VideoStream.id == stream_id).first()
    if not stream:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Video stream with ID {stream_id} not found"
        )
    result = VideoStreamResponse(
        id=stream.id,
        source_type=stream.source_type,
        source_id=stream.source_id,
        recording_path=stream.recording_path,
        last_checked=stream.last_checked,
        status=stream.status,
        url=stream.url,
        recording_started_at=stream.recording_started_at,
        recording_ended_at=stream.recording_ended_at
    )
    return result

logger = logging.getLogger("robodine.streaming_service")
router = APIRouter()

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
def add_stream(stream_data: VideoStreamRequest,
                db: Session = Depends(get_db)):
    url = stream_data.url
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


    # 새 스트림 생성
    new_stream = VideoStream(
        source_type=stream_data.source_type,
        source_id=stream_data.source_id,
        url=stream_data.url,
        recording_path=get_file_path(stream_data.url, stream_data.recording_path),
        recording_started_at=stream_data.recording_started_at,
        recording_ended_at=stream_data.recording_ended_at,
        last_checked=datetime.utcnow(),
        status=StreamStatus.ACTIVE
    )
    
    db.add(new_stream)
    db.commit()
    
    # 스트리밍 서비스에 URL 추가
    add_stream_url(stream_data.url, stream_data.recording_path)
        
    logger.info(f"[STREAM] Started recording thread for {url}")
    return {"added": url}

@router.post("/remove")
def remove_stream(req: VideoStreamRequest):
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