from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import List
from datetime import datetime
from pydantic import BaseModel
from sqlmodel import SQLModel, Field, Column

from app.core.db_config import get_db
from app.models import Table, GroupAssignment, Customer
from app.models.enums import TableStatus, LogLevel
from app.models.face_recognition import FaceRecognition

router = APIRouter()

# 로거 설정 및 저장
import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.FileHandler('inventory.log')
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)
# Create a directory for logs if it doesn't exist
import os
LOG_DIR = os.path.join(os.path.dirname(__file__), '..','..', '..', 'logs')
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, 'inventory.log')
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, 'w') as f:
        f.write("Inventory log file created.\n")
    f.write("Log entries will be appended here.\n")
    f.close()

# --- Table Models ---
class FaceResponse(BaseModel):
    id: int
    table_id: int
    timestamp: datetime
    history: str  # JSON string representation of a list
    nowdetected: int
    reliability: int
    exist: int  # 0: 없음, 1: 있음

class FaceCreateRequest(BaseModel):
    table_id: int
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    history: str  # JSON string representation of a list
    nowdetected: int
    reliability: int
    exist: int  # 0: 없음, 1: 있음

# --- Router Endpoints ---
@router.get("", response_model=List[FaceResponse])
def get_tables(db: Session = Depends(get_db)):
    faces = db.query(FaceRecognition).all()
    return [
        FaceResponse(
            id=face.id,
            timestamp=face.timestamp,
            history=face.history,
            nowdetected=face.nowdetected,
            reliability=face.reliability,
            exist=face.exist,
            table_id=face.table_id
        )
        for face in faces
    ]

@router.post("", response_model=FaceResponse)
def create_table(
    face_data: FaceCreateRequest,
    db: Session = Depends(get_db)
):
    # Create new table
    new_face = FaceRecognition(
        table_id=face_data.table_id,
        timestamp=face_data.timestamp,
        history=face_data.history,
        nowdetected=face_data.nowdetected,
        reliability=face_data.reliability,
        exist=face_data.exist,
    )
    db.add(new_face)
    db.commit()
    db.refresh(new_face)
    
    return FaceResponse(
        id=new_face.id,
        table_id=new_face.table_id,
        timestamp=new_face.timestamp,
        history=new_face.history,
        nowdetected=new_face.nowdetected,
        reliability=new_face.reliability,
        exist=new_face.exist
    )
