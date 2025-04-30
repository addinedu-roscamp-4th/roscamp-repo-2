from sqlmodel import SQLModel, Field, Column
from typing import Optional
from datetime import datetime
from sqlalchemy import Enum as SQLEnum
from .enums import EventType, EntityType

class Event(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    type: Optional[EventType] = Field(sa_column=Column(SQLEnum(EventType)))
    related_entity_type: Optional[EntityType] = Field(sa_column=Column(SQLEnum(EntityType)))
    related_entity_id: Optional[str] = None
    description: Optional[str] = None
    timestamp: datetime = Field(default_factory=datetime.utcnow)

class SystemLog(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    level: Optional[str] = None
    message: Optional[str] = None
    timestamp: datetime = Field(default_factory=datetime.utcnow)
