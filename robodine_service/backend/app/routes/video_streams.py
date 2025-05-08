# robodine_service/backend/app/routes/video_streams.py

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
from app.services.streaming_service import add_stream_url, get_file_path, _active_streams, _average_latencies, remove_stream_url  # 여기서 _active_streams와 _average_latencies를 import

logger = logging.getLogger("robodine.streaming_service")
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

class VideoRemoveRequest(BaseModel):
    url: str

class VideoDeleteRequest(BaseModel):
    id: int

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

@router.post("/delete", response_model=dict)
def remove_video_stream(
    stream_data: VideoRemoveRequest,
    db: Session = Depends(get_db)
):
    # 스트림 삭제
    stream = db.query(VideoStream).filter(VideoStream.id == stream_data.id).first()
    if not stream:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Video stream with ID {stream_data.id} not found"
        )
    if stream:
        # 상태 업데이트
        stream.status = StreamStatus.INACTIVE
        stream.last_checked = datetime.utcnow()
        db.add(stream)
        db.commit()
    
    # # 스트리밍 서비스에서 URL 제거
    # remove_stream_url(stream_data.url)
    
    return {"status": "success", "message": "스트림이 삭제되었습니다.", "stream_id": stream_data.url}

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

@router.post("/add")
def add_stream(stream_data: VideoStreamRequest,
                db: Session = Depends(get_db)):
    url = stream_data.url
    if url in _active_streams:
        return {"status": "success", "message": f"Stream {url} is already active."}
    
    # 스트리밍 서비스에 URL 추가
    add_stream_url(stream_data.url, stream_data.recording_path)

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
        
    logger.info(f"[STREAM] Started recording thread for {url}")
    return {"added": url}


@router.post("/remove")
def remove_stream(req: VideoRemoveRequest,
                db: Session = Depends(get_db)):
    url = req.url
    info = _active_streams.pop(url, None)
    
    if not info:
        # 이미 중지된 URL 도 404로 처리
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"Stream {url} not found.")
    
    # 스트리밍 서비스에서 URL 제거
    remove_stream_url(url)

    # 중지 시그널
    info["stop_event"].set()
    # 최대 5초 대기
    info["thread"].join(timeout=5)

    avg = _average_latencies.pop(url, None)
    return {"status": "success", "message": "스트림이 비활성화되었습니다.", "average_latency": avg, "url": url}
