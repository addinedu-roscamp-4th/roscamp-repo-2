from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import Optional, List
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Albabot, Robot
from app.models.enums import RobotStatus, EntityType, LogLevel
from app.routes.events import log_info, log_warning, log_error

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
    albabot = db.query(Albabot).filter(Albabot.robot_id == int(robot_id)).first()
    
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
    existing_albabot = db.query(Albabot).filter(Albabot.robot_id == int(albabot.robot_id)).order_by(Albabot.id.desc()).first()
    if existing_albabot:
        prev_status = existing_albabot.status
        prev_battery = existing_albabot.battery_level
    
    # Create new Albabot record
    new_albabot = Albabot(
        robot_id=int(albabot.robot_id),
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
        if prev_status == RobotStatus.IDLE:
            log_message += f" (이전 상태: 대기)"
        elif prev_status == RobotStatus.SERVING:
            log_message += f" (이전 상태: 서빙 중)"
        elif prev_status == RobotStatus.CLEANING:
            log_message += f" (이전 상태: 청소 중)"
        elif prev_status == RobotStatus.EMERGENCY:
            log_message += f" (이전 상태: 비상)"
        elif prev_status == RobotStatus.SECURITY:
            log_message += f" (이전 상태: 보안 모드)"
        elif prev_status == RobotStatus.CHARGING:
            log_message += f" (이전 상태: 충전 중)"
        elif prev_status == RobotStatus.ERROR:
            log_message += f" (이전 상태: 오류)"
        else:
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
    if log_level == LogLevel.INFO:
        log_info(db, log_message, background_tasks)
    elif log_level == LogLevel.WARNING:
        log_warning(db, log_message, background_tasks)
    elif log_level == LogLevel.ERROR:
        log_error(db, log_message, background_tasks)
    else:
        log_info(db, log_message, background_tasks)
    
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

@router.post("/{robot_id}/command", response_model=dict)
def send_command_to_albabot(
    robot_id: int, 
    command_data: dict, 
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    """알바봇에 명령 전송 (ex: 주문 서빙, 청소 등)"""
    # Check if robot exists
    robot = db.query(Robot).filter(Robot.robot_id == robot_id).first()
    if not robot:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Robot with ID {robot_id} not found"
        )
    
    # Check if it's an Albabot
    if robot.type != EntityType.ALBABOT:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Robot {robot_id} is not an Albabot"
        )
    
    # Get latest Albabot status
    albabot = db.query(Albabot).filter(Albabot.robot_id == robot_id).order_by(Albabot.id.desc()).first()
    if not albabot:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Albabot data for robot {robot_id} not found"
        )
    
    # Check if robot is available
    if albabot.status not in [RobotStatus.IDLE]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Albabot {robot_id} is not available (current status: {albabot.status})"
        )
    
    # Process command
    command_type = command_data.get("type", "").upper()
    
    # Create new status entry
    new_status = None
    if command_type == "SERVE":
        new_status = RobotStatus.SERVING
    elif command_type == "CLEAN":
        new_status = RobotStatus.CLEANING
    else:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid command type: {command_type}"
        )
    
    # Create new albabot entry with updated status
    new_albabot = Albabot(
        robot_id=robot_id,
        status=new_status,
        battery_level=albabot.battery_level,
        timestamp=datetime.utcnow()
    )
    
    db.add(new_albabot)
    db.commit()
    db.refresh(new_albabot)
    
    # Log this action
    log_info(db, f"알바봇 {robot_id}에 {command_type} 명령 전송, 상태 변경: {albabot.status} → {new_status}", background_tasks)
    
    # Return success response
    return {
        "status": "success",
        "message": f"Command {command_type} sent to Albabot {robot_id}",
        "robot_id": robot_id,
        "new_status": str(new_status)
    }