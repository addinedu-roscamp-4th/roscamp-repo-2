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
