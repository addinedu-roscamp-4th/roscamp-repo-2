from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class Event(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    type: Optional[str] = None
    description: Optional[str] = None
    timestamp: Optional[datetime] = None
