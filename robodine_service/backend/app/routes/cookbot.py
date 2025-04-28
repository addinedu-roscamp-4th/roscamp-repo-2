from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Cookbot
from app.models.enums import RobotStatus

router = APIRouter()

class CookbotStatusResponse(BaseModel):
    robot_id: int
    status: RobotStatus
    timestamp: datetime

@router.get("/status/{robot_id}", response_model=CookbotStatusResponse)
def get_cookbot_status(robot_id: int, db: Session = Depends(get_db)):
    # Find Cookbot record
    cookbot = db.query(Cookbot).filter(Cookbot.robot_id == str(robot_id)).first()
    
    if not cookbot:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Cookbot with ID {robot_id} not found"
        )
    
    return CookbotStatusResponse(
        robot_id=int(cookbot.robot_id),
        status=cookbot.status,
        timestamp=cookbot.timestamp
    ) 