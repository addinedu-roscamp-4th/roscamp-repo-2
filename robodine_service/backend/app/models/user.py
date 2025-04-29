from sqlmodel import SQLModel, Field, Relationship, Column
from typing import Optional, List
from datetime import datetime
from sqlalchemy import Enum as SQLEnum
from .enums import UserRole, NotificationStatus
from pydantic import EmailStr

class UserBase(SQLModel):
    username: str = Field(index=True)
    email: Optional[EmailStr] = None
    full_name: str
    role: UserRole = Field(default=UserRole.EMPLOYEE, sa_column=Column(SQLEnum(UserRole)))

class UserCreate(UserBase):
    password: str

class UserRead(UserBase):
    id: int
    created_at: datetime

class User(UserBase, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    password: str  # Hashed password
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    last_login: Optional[datetime] = None
    is_active: bool = True
    
    notifications: List["Notification"] = Relationship(back_populates="user")

class Notification(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    user_id: Optional[int] = Field(default=None, foreign_key="user.id")
    message: str
    type: str
    created_at: datetime = Field(default_factory=datetime.utcnow)
    status: Optional[NotificationStatus] = Field(sa_column=Column(SQLEnum(NotificationStatus)))
    
    user: Optional[User] = Relationship(back_populates="notifications")

class Token(SQLModel):
    access_token: str
    token_type: str 