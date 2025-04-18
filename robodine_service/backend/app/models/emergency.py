from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class Emergency(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    robot_id: Optional[str] = Field(default=None, foreign_key="robot.id")
    type: Optional[str] = None
    description: Optional[str] = None
    timestamp: Optional[datetime] = None
