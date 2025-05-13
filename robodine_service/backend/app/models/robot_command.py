from typing import Optional, Dict, TYPE_CHECKING
from sqlmodel import SQLModel, Field, Relationship, Column
from sqlalchemy import JSON, Enum as SQLEnum
from datetime import datetime
from .enums import CommandStatus

if TYPE_CHECKING:
    from .robot import Robot

class RobotCommand(SQLModel, table=True):
    """
    로봇 명령 정보 관리 모델
    """
    __tablename__ = "robotcommand"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    robot_id: Optional[int] = Field(default=None, foreign_key="robot.id")
    command: Optional[str] = None
    parameters: Optional[Dict] = Field(sa_column=Column(JSON))
    status: Optional[CommandStatus] = Field(sa_column=Column(SQLEnum(CommandStatus, name="commandstatus")))
    timestamp: Optional[datetime] = Field(default_factory=datetime.utcnow)
    executed_at: Optional[datetime] = None
    
    # Relationship
    robot: Optional["Robot"] = Relationship(back_populates="commands")
