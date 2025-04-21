from sqlmodel import SQLModel, Field, Column
from sqlalchemy import JSON  # JSON 타입을 사용
from typing import Optional, Dict
from datetime import datetime

class AdminSettings(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    operation_start: Optional[str] = None
    operation_end: Optional[str] = None
    inventory_threshold: Optional[int] = None
    alert_settings: Optional[Dict] = Field(sa_column=Column(JSON))
