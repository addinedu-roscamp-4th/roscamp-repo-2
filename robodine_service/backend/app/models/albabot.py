from typing import Optional, List, TYPE_CHECKING
from datetime import datetime

from sqlmodel import SQLModel, Field, Relationship
from sqlalchemy import Column, Enum as SQLEnum

from .enums import RobotStatus, EntityType

if TYPE_CHECKING:
    from .pose6d import Pose6D
    from .order import Order

class Albabot(SQLModel, table=True):
    __tablename__ = "albabot"

    id: Optional[int] = Field(default=None, primary_key=True)
    robot_id: Optional[str] = Field(default=None, index=True)
    status: Optional[RobotStatus] = Field(
        sa_column=Column(SQLEnum(RobotStatus, name="robot_status"))
    )
    battery_level: Optional[float] = Field
    timestamp: datetime = Field(default_factory=datetime.utcnow)