from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class CleaningTask(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    robot_id: Optional[str] = Field(default=None, foreign_key="robot.id")
    table_id: Optional[int] = Field(default=None, foreign_key="table.id")
    status: Optional[str] = None
    scheduled_time: Optional[datetime] = None
    completed_time: Optional[datetime] = None
