from sqlmodel import SQLModel, Field, Relationship
from typing import Optional, List
from datetime import datetime

class Customer(SQLModel, table=True):
    """
    고객 그룹 단위로 관리: 성인/어린이 수, 대기·주문 내역
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    count: Optional[int] = Field(default=None)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    
    assignments: List["GroupAssignment"] = Relationship(back_populates="customer")
    waiting: Optional["WaitingList"] = Relationship(back_populates="customer")
    orders: List["Order"] = Relationship(back_populates="customer")
