# models/robot.py
from typing import Optional, List, TYPE_CHECKING
from datetime import datetime

from sqlmodel import SQLModel, Field, Relationship
from sqlalchemy import Column, Enum as SQLEnum, and_
from sqlalchemy.orm import foreign

from .enums import RobotStatus, EntityType

if TYPE_CHECKING:
    from .order import Order
    from .pose6d import Pose6D

class Robot(SQLModel, table=True):
    __tablename__ = "robot"

    id: Optional[int] = Field(default=None, primary_key=True)  # int PK
    type: Optional[EntityType] = Field(
        sa_column=Column(SQLEnum(EntityType, name="entity_type"))
    )
    mac_address: Optional[str] = None
    ip_address: Optional[str] = None
    status: Optional[RobotStatus] = Field(
        sa_column=Column(SQLEnum(RobotStatus, name="robot_status"))
    )
    battery_level: Optional[int] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    orders: List["Order"] = Relationship(back_populates="robot")
    commands: List["RobotCommand"] = Relationship(back_populates="robot")
    cleaning_tasks: List["CleaningTask"] = Relationship(back_populates="robot")
