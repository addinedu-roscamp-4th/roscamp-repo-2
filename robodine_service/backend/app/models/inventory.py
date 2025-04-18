from sqlmodel import SQLModel, Field
from datetime import datetime
from typing import Optional

class Inventory(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    item: Optional[str] = None
    count: Optional[int] = None
    status: Optional[str] = None
    last_updated: Optional[datetime] = None
