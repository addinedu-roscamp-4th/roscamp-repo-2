from sqlmodel import SQLModel, Field, Column
from typing import Optional
from datetime import datetime
from sqlalchemy import Enum as SQLEnum
from .enums import StreamSourceType, StreamStatus

class VideoStream(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    source_type: Optional[StreamSourceType] = Field(sa_column=Column(SQLEnum(StreamSourceType)))
    source_id: Optional[str] = None
    url: Optional[str] = None
    last_checked: datetime = Field(default_factory=datetime.utcnow)
    status: Optional[StreamStatus] = Field(sa_column=Column(SQLEnum(StreamStatus)))
