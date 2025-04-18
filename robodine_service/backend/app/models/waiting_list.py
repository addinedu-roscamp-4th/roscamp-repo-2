from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class WaitingList(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    customer_name: Optional[str] = None
    party_size: Optional[int] = None
    phone: Optional[str] = None
    check_in_time: Optional[datetime] = None
