from sqlmodel import SQLModel, Field, Relationship, Column
from typing import Optional, List
from datetime import datetime
from sqlalchemy import Enum as SQLEnum
from .enums import UserRole, NotificationStatus

class User(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    username: str = Field(index=True, unique=True)
    password_hash: str
    name: str
    role: UserRole = Field(sa_column=Column(SQLEnum(UserRole)))
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    last_login: Optional[datetime] = None
    
    notifications: List["Notification"] = Relationship(back_populates="user")

class Notification(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    user_id: Optional[int] = Field(default=None, foreign_key="user.id")
    type: Optional[str] = None
    message: Optional[str] = None
    created_at: Optional[datetime] = Field(default_factory=datetime.utcnow)
    status: Optional[NotificationStatus] = Field(sa_column=Column(SQLEnum(NotificationStatus)))
    
    user: Optional[User] = Relationship(back_populates="notifications") 