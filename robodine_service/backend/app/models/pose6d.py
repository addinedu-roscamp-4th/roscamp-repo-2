from typing import Optional, TYPE_CHECKING
from datetime import datetime

from sqlmodel import SQLModel, Field, Relationship
from sqlalchemy import Column, Enum as SQLEnum

from .enums import EntityType

if TYPE_CHECKING:
    from .robot import Robot

class Pose6D(SQLModel, table=True):
    __tablename__ = "pose6d"

    id: Optional[int] = Field(default=None, primary_key=True)
    entity_id: int = Field(default=None)
    # Store the type of entity (e.g. Robot) to which this pose belongs
    entity_type: EntityType = Field(
        sa_column=Column(
            SQLEnum(EntityType, name="entity_type"),
            nullable=False
        )
    )

    timestamp: datetime = Field(default_factory=datetime.utcnow)

    # Pose fields
    x: float
    y: float
    z: float

    roll: float
    pitch: float
    yaw: float