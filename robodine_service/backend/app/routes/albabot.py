from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Albabot
from app.models.enums import RobotStatus

router = APIRouter()

class AlbabotStatusResponse(BaseModel):
    robot_id: int
    status: RobotStatus
    battery_level: str
    timestamp: datetime

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
        battery_level=str(albabot.battery_level),
        timestamp=albabot.timestamp
    ) 