from sqlmodel import SQLModel, Field, Relationship
from typing import Optional
from datetime import datetime
from .robot import Robot

class CleaningTask(SQLModel, table=True):
    """
    청소 작업 정보 관리 모델
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    robot_id: Optional[int] = Field(default=None, foreign_key="robot.id")
    area: Optional[str] = None
    status: Optional[str] = None
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    
    # Relationship
    robot: Optional[Robot] = Relationship(back_populates="cleaning_tasks")
