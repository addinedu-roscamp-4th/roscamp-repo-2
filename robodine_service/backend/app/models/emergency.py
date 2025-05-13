from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class Emergency(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    emergency_type: Optional[str] = None
    description: Optional[str] = None
    is_active: bool = Field(default=True)
    timestamp: Optional[datetime] = Field(default_factory=datetime.utcnow)
    resolved_at: Optional[datetime] = None
