from datetime import datetime
from typing import Optional
from enum import Enum, auto
from sqlmodel import Field, SQLModel
from app.models.enums import ChatStatus

class Chat(SQLModel, table=True):
    """
    채팅 메시지 모델
    
    사용자와 AI 간 대화 메시지를 저장
    """
    __tablename__ = "chat"
    # id는 자동 증가하는 primary key로 설정
    id: Optional[int] = Field(default=None, primary_key=True)
    question: str = Field(...)
    answer: Optional[str] = Field(default=None)
    timestamp: datetime = Field(default_factory=datetime.now)
    robot_id: Optional[int] = Field(default=None, foreign_key="robot.id", index=True)
    robot_task: Optional[str] = Field(default=None)
    status: Optional[ChatStatus] = Field(default=ChatStatus.PENDING)
    table_id: Optional[int] = Field(default=None)