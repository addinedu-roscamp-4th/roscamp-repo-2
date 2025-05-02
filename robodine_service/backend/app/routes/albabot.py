from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import Optional, List
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Albabot
from app.models.enums import RobotStatus, LogLevel
from app.models.event import SystemLog

router = APIRouter()

class AlbabotStatusResponse(BaseModel):
    robot_id: int
    status: RobotStatus
    battery_level: int
    timestamp: datetime

@router.get("", response_model=List[AlbabotStatusResponse])
def get_all_albabot_status(db: Session = Depends(get_db)):
    # robot_id가 겹치지 않는 Albabot 레코드들 가져오기
    albabot_records = (
        db.query(Albabot)
        .order_by(Albabot.robot_id, Albabot.id.desc())
        .distinct(Albabot.robot_id)
        .all()
    )
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
def create_albabot_status(
    albabot: AlbabotStatusResponse,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
    ):
    # 이전 상태 조회
    prev_status = None
    prev_battery = None
    existing_albabot = db.query(Albabot).filter(Albabot.robot_id == str(albabot.robot_id)).order_by(Albabot.id.desc()).first()
    if existing_albabot:
        prev_status = existing_albabot.status
        prev_battery = existing_albabot.battery_level
    
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
    
    # 시스템 로그 생성 (상태와 배터리 레벨에 따라 다른 메시지와 레벨 설정)
    log_level = LogLevel.INFO
    status_changed = prev_status is not None and prev_status != albabot.status
    battery_changed = prev_battery is not None and abs(float(prev_battery) - float(albabot.battery_level)) > 0.05
    
    # 상태 변경에 따른 로그 메시지
    if status_changed:
        if albabot.status == RobotStatus.IDLE:
            log_message = f"알바봇 #{albabot.robot_id}가 대기 상태로 변경되었습니다."
        elif albabot.status == RobotStatus.SERVING:
            log_message = f"알바봇 #{albabot.robot_id}가 서빙 중입니다."
        elif albabot.status == RobotStatus.CLEANING:
            log_message = f"알바봇 #{albabot.robot_id}가 청소 중입니다."
        elif albabot.status == RobotStatus.EMERGENCY:
            log_level = LogLevel.ERROR
            log_message = f"알바봇 #{albabot.robot_id}가 비상 상태입니다."
        elif albabot.status == RobotStatus.SECURITY:
            log_level = LogLevel.WARNING
            log_message = f"알바봇 #{albabot.robot_id}가 보안 모드로 전환되었습니다."
        elif albabot.status == RobotStatus.CHARGING:
            log_message = f"알바봇 #{albabot.robot_id}가 충전 중입니다."
        elif albabot.status == RobotStatus.ERROR:
            log_level = LogLevel.ERROR
            log_message = f"알바봇 #{albabot.robot_id}에 오류가 발생했습니다."
        else:
            log_message = f"알바봇 #{albabot.robot_id}의 상태가 '{albabot.status}'로 변경되었습니다."
        
        # 이전 상태 정보 추가
        log_message += f" (이전 상태: {prev_status})"
    # 배터리 변경에 따른 로그 메시지
    elif battery_changed:
        # 배터리 퍼센트로 변환 (0-1 범위이면 100 곱함)
        battery_percent = float(albabot.battery_level)
        if battery_percent <= 1:
            battery_percent *= 100
        
        # 배터리 레벨에 따른 로그 레벨 조정
        if battery_percent <= 30:
            log_level = LogLevel.WARNING
            log_message = f"알바봇 #{albabot.robot_id}의 배터리가 부족합니다. (배터리: {battery_percent:.0f}%)"
        elif battery_percent <= 10:
            log_level = LogLevel.ERROR
            log_message = f"알바봇 #{albabot.robot_id}의 배터리가 심각하게 부족합니다. (배터리: {battery_percent:.0f}%)"
        else:
            log_message = f"알바봇 #{albabot.robot_id}의 배터리 레벨이 변경되었습니다. (배터리: {battery_percent:.0f}%)"
    else:
        # 특별한 변경이 없는 경우
        log_message = f"알바봇 #{albabot.robot_id}의 상태가 업데이트되었습니다. (상태: {albabot.status})"
    
    # 시스템 로그 저장
    log = SystemLog(
        level=log_level,
        message=log_message,
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    db.refresh(log)
    
    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "albabot",
        None
    )
    # 시스템 로그 브로드캐스트 예약
    background_tasks.add_task(
        broadcast_entity_update,
        "systemlog",
        None
    )

    return AlbabotStatusResponse(
        robot_id=int(new_albabot.robot_id),
        status=new_albabot.status,
        battery_level=int(new_albabot.battery_level),
        timestamp=new_albabot.timestamp
    )