from typing import Optional, List, TYPE_CHECKING
from datetime import datetime

from sqlmodel import SQLModel, Field, Relationship
from sqlalchemy import Column, Enum as SQLEnum

from .enums import RobotStatus, EntityType

if TYPE_CHECKING:
    from .pose6d import Pose6D
    from .order import Order

class Robot(SQLModel, table=True):
    __tablename__ = "robot"

    id: Optional[int] = Field(default=None, primary_key=True)
    robot_id: Optional[str] = Field(default=None, index=True)
    type: Optional[EntityType] = Field(
        sa_column=Column(SQLEnum(EntityType, name="entity_type"))
    )
    mac_address: Optional[str] = None
    ip_address: Optional[str] = None
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    orders: List["Order"]            = Relationship(back_populates="robot")
    commands: List["RobotCommand"]   = Relationship(back_populates="robot")
    cleaning_tasks: List["CleaningTask"] = Relationship(back_populates="robot")

    # ← No positional args here
    poses: List["Pose6D"] = Relationship(
        back_populates="robot",
        sa_relationship_kwargs={
            # string‐based join to avoid NameErrors at class‐def time
            "primaryjoin": (
                "and_("
                "robot.c.id == pose6d.c.entity_id, "
                "pose6d.c.entity_type == 'ALBABOT'"
                ")"
            ),
            "cascade": "all, delete-orphan",
            "lazy": "selectin",
        },
    )
