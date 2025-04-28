from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import VideoStream, User
from app.models.enums import StreamSourceType, StreamStatus, UserRole
from app.routes.auth import get_current_user

router = APIRouter()

# --- Video Stream Models ---
class VideoStreamResponse(BaseModel):
    id: int
    source_type: StreamSourceType
    source_id: str
    url: str
    last_checked: datetime
    status: StreamStatus

class VideoStreamUpdateRequest(BaseModel):
    url: str

# --- Router Endpoints ---
@router.get("", response_model=List[VideoStreamResponse])
def get_video_streams(db: Session = Depends(get_db)):
    streams = db.query(VideoStream).all()
    return [
        VideoStreamResponse(
            id=stream.id,
            source_type=stream.source_type,
            source_id=stream.source_id,
            url=stream.url,
            last_checked=stream.last_checked,
            status=stream.status
        ) for stream in streams
    ]

@router.put("/{stream_id}/refresh", response_model=dict)
def refresh_stream(
    stream_id: int,
    stream_data: VideoStreamUpdateRequest,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can update stream URLs
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to update stream URLs"
        )
    
    # Find stream
    stream = db.query(VideoStream).filter(VideoStream.id == stream_id).first()
    if not stream:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Video stream with ID {stream_id} not found"
        )
    
    # Update stream URL
    stream.url = stream_data.url
    stream.last_checked = datetime.utcnow()
    stream.status = StreamStatus.ACTIVE
    
    db.add(stream)
    db.commit()
    
    return {"status": "success", "message": "스트림 URL이 갱신되었습니다."} 