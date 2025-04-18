from sqlmodel import SQLModel, Field
from sqlalchemy import Column
from sqlalchemy.dialects.postgresql import JSON  # JSON 타입을 사용
from typing import Optional
from datetime import datetime

class AdminSettings(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    setting_key: Optional[str] = None
    setting_value: Optional[str] = None
    category: Optional[str] = None
    description: Optional[str] = None
    updated_at: Optional[datetime] = None
    inventory_threshold: Optional[int]
    operation_start: Optional[datetime] = None
    operation_end: Optional[datetime] = None
    alerts: Optional[str] = None
