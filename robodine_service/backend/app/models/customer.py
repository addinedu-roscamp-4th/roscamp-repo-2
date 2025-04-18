from sqlmodel import SQLModel, Field, Relationship
from typing import Optional, List
from datetime import datetime

class Customer(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    name: Optional[str] = None
    phone: Optional[str] = None
    table_number: Optional[int] = None
    entry_time: Optional[datetime] = None
    exit_time: Optional[datetime] = None

    # Optional relationships
    orders: Optional[List["Order"]] = Relationship(back_populates="customer")
