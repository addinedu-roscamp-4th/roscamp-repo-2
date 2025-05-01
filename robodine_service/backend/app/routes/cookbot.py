from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import Optional, List
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

@router.get("", response_model=List[CookbotStatusResponse])
def get_all_albabot_status(db: Session = Depends(get_db)):
    # robot_id가 겹치지 않는 Albabot 레코드들 가져오기
    cookbot_records = (
        db.query(Cookbot)
        .order_by(Cookbot.robot_id, Cookbot.id.desc())
        .distinct(Cookbot.robot_id)
        .all()
    )
    if not cookbot_records:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="No Cookbot records found"
        )
    cookbot_statuses = []
    for cookbot in cookbot_records:
        cookbot_statuses.append(
            CookbotStatusResponse(
                robot_id=int(cookbot.robot_id),
                status=cookbot.status,
                timestamp=cookbot.timestamp
            )
        )
    return cookbot_statuses
    

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

@router.post("/status", response_model=CookbotStatusResponse)
def create_cookbot_status(
    cookbot_in: CookbotStatusResponse,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # 새로운 Cookbot 상태 기록 생성
    new_cookbot = Cookbot(
        robot_id=str(cookbot_in.robot_id),
        status=cookbot_in.status,
        timestamp=cookbot_in.timestamp
    )
    db.add(new_cookbot)
    db.commit()
    db.refresh(new_cookbot)

    # 모듈 초기화 이후 동적 import
    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "cookbot",
        int(new_cookbot.robot_id)
    )

    return CookbotStatusResponse(
        robot_id=int(new_cookbot.robot_id),
        status=new_cookbot.status,
        timestamp=new_cookbot.timestamp
    )
