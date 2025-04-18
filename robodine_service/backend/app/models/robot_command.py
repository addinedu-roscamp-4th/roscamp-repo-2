from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class RobotCommand(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    robot_id: Optional[str] = Field(default=None, foreign_key="robot.id")
    command_type: Optional[str] = None
    parameters: Optional[str] = None  # JSON 문자열로 저장된 파라미터
    status: Optional[str] = None  # pending, executing, completed, failed 등
    created_at: Optional[datetime] = None
    executed_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    response: Optional[str] = None  # JSON 문자열로 저장된 응답
