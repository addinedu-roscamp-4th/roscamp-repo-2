from sqlmodel import SQLModel, Field, Column
from typing import Optional
from datetime import datetime
from sqlalchemy import Enum as SQLEnum
from .enums import StreamSourceType, StreamStatus

class VideoStream(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    source_type: Optional[StreamSourceType] = Field(sa_column=Column(SQLEnum(StreamSourceType)))
    source_id: Optional[str] = None
    last_checked: Optional[datetime] = Field(default_factory=datetime.utcnow)
    recording_started_at: Optional[datetime] = None
    recording_ended_at: Optional[datetime] = None
    url: Optional[str] = None
    status: Optional[StreamStatus] = Field(sa_column=Column(SQLEnum(StreamStatus)))
    recording_path: Optional[str] = None
