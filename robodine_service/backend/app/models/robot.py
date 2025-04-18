from typing import Optional, List
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime

class Robot(SQLModel, table=True):
    id: str = Field(primary_key=True)  # Only ID is required
    mac_address: Optional[str] = None
    ip_address: Optional[str] = None
    status: Optional[str] = None
    location: Optional[str] = None
    battery_level: Optional[int] = None
    timestamp: Optional[datetime] = None
    
    # Optional relationships
    orders: Optional[List["Order"]] = Relationship(back_populates="robot")
