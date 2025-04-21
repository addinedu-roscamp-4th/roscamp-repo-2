from sqlmodel import SQLModel, Field, Relationship
from typing import Optional
from datetime import datetime

class WaitingList(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    customer_id: Optional[int] = Field(default=None, foreign_key="customer.id")
    since: datetime = Field(default_factory=datetime.utcnow)
    estimated_wait: Optional[int] = None
    
    customer: Optional["Customer"] = Relationship(back_populates="waiting")
