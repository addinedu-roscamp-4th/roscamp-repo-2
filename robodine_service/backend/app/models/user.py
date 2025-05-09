from sqlmodel import SQLModel, Field, Relationship, Column
from typing import Optional, List
from datetime import datetime
from sqlalchemy import Enum as SQLEnum
from .enums import UserRole, NotificationStatus

class User(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    username: Optional[str] = None
    password_hash: Optional[str] = None
    role: Optional[UserRole] = Field(sa_column=Column(SQLEnum(UserRole)))
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    
    notifications: List["Notification"] = Relationship(back_populates="user")

class Notification(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    user_id: Optional[int] = Field(default=None, foreign_key="user.id")
    type: Optional[str] = None
    message: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    status: Optional[NotificationStatus] = Field(sa_column=Column(SQLEnum(NotificationStatus)))
    
    user: Optional[User] = Relationship(back_populates="notifications") 