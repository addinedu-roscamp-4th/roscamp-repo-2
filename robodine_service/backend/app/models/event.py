from sqlmodel import SQLModel, Field, Column
from typing import Optional
from datetime import datetime
from sqlalchemy import Enum as SQLEnum
from enum import Enum
from .enums import EventType, EntityType

# 로그 레벨 열거형 추가
class LogLevel(str, Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    DEBUG = "DEBUG"

class Event(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    type: Optional[EventType] = Field(sa_column=Column(SQLEnum(EventType)))
    related_entity_type: Optional[EntityType] = Field(sa_column=Column(SQLEnum(EntityType)))
    related_entity_id: Optional[str] = None
    description: Optional[str] = None
    timestamp: Optional[datetime] = Field(default_factory=datetime.utcnow)

class SystemLog(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    level: Optional[LogLevel] = Field(sa_column=Column(SQLEnum(LogLevel)))
    message: Optional[str] = None
    timestamp: Optional[datetime] = Field(default_factory=datetime.utcnow)
