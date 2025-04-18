from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class Table(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    table_number: Optional[int] = None
    capacity: Optional[int] = None
    status: Optional[str] = None
    last_occupied: Optional[datetime] = None
    location_x: Optional[float] = None
    location_y: Optional[float] = None
