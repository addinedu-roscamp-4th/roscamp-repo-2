from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional, List
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Albabot
from app.models.enums import RobotStatus

router = APIRouter()

class AlbabotStatusResponse(BaseModel):
    robot_id: int
    status: RobotStatus
    battery_level: int
    timestamp: datetime

@router.get("", response_model=List[AlbabotStatusResponse])
def get_all_albabot_status(db: Session = Depends(get_db)):
    # robot_id가 겹치지 않는 Albabot 레코드들 가져오기
    albabot_records = db.query(Albabot).distinct(Albabot.robot_id).all()
    if not albabot_records:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="No Albabot records found"
        )
    albabot_statuses = []
    for albabot in albabot_records:
        albabot_statuses.append(
            AlbabotStatusResponse(
                robot_id=int(albabot.robot_id),
                status=albabot.status,
                battery_level=int(albabot.battery_level),
                timestamp=albabot.timestamp
            )
        )
    return albabot_statuses


@router.get("/status/{robot_id}", response_model=AlbabotStatusResponse)
def get_albabot_status(robot_id: int, db: Session = Depends(get_db)):
    # Find Albabot record
    albabot = db.query(Albabot).filter(Albabot.robot_id == str(robot_id)).first()
    
    if not albabot:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Albabot with ID {robot_id} not found"
        )
    
    return AlbabotStatusResponse(
        robot_id=int(albabot.robot_id),
        status=albabot.status,
        battery_level=int(albabot.battery_level),
        timestamp=albabot.timestamp
    ) 

@router.post("/status", response_model=AlbabotStatusResponse)
def create_albabot_status(albabot: AlbabotStatusResponse, db: Session = Depends(get_db)):
    # Check if Albabot with the same robot_id already exists
    existing_albabot = db.query(Albabot).filter(Albabot.robot_id == str(albabot.robot_id)).first()
    # Create new Albabot record
    new_albabot = Albabot(
        robot_id=str(albabot.robot_id),
        status=albabot.status,
        battery_level=albabot.battery_level,
        timestamp=albabot.timestamp
    )
    
    db.add(new_albabot)
    db.commit()
    db.refresh(new_albabot)
    
    return AlbabotStatusResponse(
        robot_id=int(new_albabot.robot_id),
        status=new_albabot.status,
        battery_level=str(new_albabot.battery_level),
        timestamp=new_albabot.timestamp
    )