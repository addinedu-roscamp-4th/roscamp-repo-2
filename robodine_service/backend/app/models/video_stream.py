from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class VideoStream(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    robot_id: Optional[str] = Field(default=None, foreign_key="robot.id")
    url: Optional[str] = None
    status: Optional[str] = None
    started_at: Optional[datetime] = None
