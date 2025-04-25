from typing import Optional, TYPE_CHECKING
from datetime import datetime

from sqlmodel import SQLModel, Field, Relationship
from sqlalchemy import Column, Enum as SQLEnum

from .enums import EntityType

if TYPE_CHECKING:
    from .robot import Robot

class JointAngles(SQLModel, table=True):
    __tablename__ = "jointangles"

    id: Optional[int] = Field(default=None, primary_key=True)
    robot_id: int = Field(default=None)
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    # Joint angles
    joint1: float
    joint2: float
    joint3: float
    joint4: float
    joint5: float
    joint6: float