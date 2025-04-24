from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class Chat(SQLModel, table=True):
    __tablename__ = "chat"
    # id는 자동 증가하는 primary key로 설정
    id: Optional[int] = Field(default=None, primary_key=True)
    question: Optional[str] = None
    answer: Optional[str] = None
    timestamp: Optional[datetime] = Field(default_factory=datetime.utcnow)
