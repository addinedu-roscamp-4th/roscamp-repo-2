# models/pose6d.py
from typing import Optional
from datetime import datetime

from sqlmodel import SQLModel, Field, Column, Relationship
from sqlalchemy import Enum as SQLEnum, and_
from sqlalchemy.orm import foreign

from .enums import EntityType

class Pose6D(SQLModel, table=True):
    __tablename__ = "pose6d"

    id: Optional[int] = Field(default=None, primary_key=True)
    entity_type: EntityType = Field(
        sa_column=Column(SQLEnum(EntityType, name="entity_type"), nullable=False)
    )
    entity_id: int = Field(sa_column=Column(nullable=False))  # 모든 엔티티 PK를 int로 통일
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float