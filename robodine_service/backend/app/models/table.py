from sqlmodel import SQLModel, Field, Relationship, Column
from typing import Optional, List
from datetime import datetime
from sqlalchemy import Enum as SQLEnum
from .enums import TableStatus

class Table(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    number: Optional[int] = None
    max_customer: Optional[int] = None
    status: Optional[TableStatus] = Field(sa_column=Column(SQLEnum(TableStatus)))
    updated_at: Optional[datetime] = Field(default_factory=datetime.utcnow)

    
    assignments: List["GroupAssignment"] = Relationship(back_populates="table")

class GroupAssignment(SQLModel, table=True):
    """
    테이블 배정 이력: assigned_at, released_at 으로 사용시간 추적
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    table_id: Optional[int] = Field(default=None, foreign_key="table.id")
    customer_id: Optional[int] = Field(default=None, foreign_key="customer.id")
    timestamp: Optional[datetime] = Field(default_factory=datetime.utcnow)
    released_at: Optional[datetime] = None
    
    table: Optional[Table] = Relationship(back_populates="assignments")
    customer: Optional["Customer"] = Relationship(back_populates="assignments")
