from typing import Optional, TYPE_CHECKING
from datetime import datetime

from sqlmodel import SQLModel, Field, Relationship
from sqlalchemy import Column, Enum as SQLEnum
import json


class FaceRecognition(SQLModel, table=True):
    __tablename__ = "face_recognition"

    id: Optional[int] = Field(default=None, primary_key=True)
    table_id: int
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    history: str  # Store as a string (JSON string representation of a list)
    nowdetected: int
    reliability: int  # 신뢰도 0~100
    exist: int  # 0: 없음, 1: 있음