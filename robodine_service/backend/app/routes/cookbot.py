from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import Optional, List
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Cookbot, Robot, Order
from app.models.enums import RobotStatus, LogLevel, OrderStatus, EntityType
from app.routes.events import log_info, log_warning, log_error

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
    # 이전 상태 조회
    prev_status = None
    prev_cookbot = db.query(Cookbot).filter(Cookbot.robot_id == str(cookbot_in.robot_id)).order_by(Cookbot.id.desc()).first()
    if prev_cookbot:
        prev_status = prev_cookbot.status

    # 새로운 Cookbot 상태 기록 생성
    new_cookbot = Cookbot(
        robot_id=str(cookbot_in.robot_id),
        status=cookbot_in.status,
        timestamp=cookbot_in.timestamp
    )
    db.add(new_cookbot)
    db.commit()
    db.refresh(new_cookbot)

    # 시스템 로그 생성 (상태에 따라 다른 메시지와 레벨 설정)
    log_level = LogLevel.INFO
    status_changed = prev_status is not None and prev_status != cookbot_in.status
    
    if cookbot_in.status == RobotStatus.IDLE:
        log_message = f"쿡봇 #{cookbot_in.robot_id}가 유휴 상태입니다."
    elif cookbot_in.status == RobotStatus.COOKING:
        log_message = f"쿡봇 #{cookbot_in.robot_id}가 요리 중입니다."
    elif cookbot_in.status == RobotStatus.SERVING:
        log_message = f"쿡봇 #{cookbot_in.robot_id}가 서빙 중입니다."
    elif cookbot_in.status == RobotStatus.CLEANING:
        log_message = f"쿡봇 #{cookbot_in.robot_id}가 청소 중입니다."
    elif cookbot_in.status == RobotStatus.EMERGENCY:
        log_level = LogLevel.ERROR
        log_message = f"쿡봇 #{cookbot_in.robot_id}가 비상 상태입니다."
    elif cookbot_in.status == RobotStatus.SECURITY:
        log_level = LogLevel.WARNING
        log_message = f"쿡봇 #{cookbot_in.robot_id}가 보안 모드입니다."
    elif cookbot_in.status == RobotStatus.CHARGING:
        log_message = f"쿡봇 #{cookbot_in.robot_id}가 충전 중입니다."
    elif cookbot_in.status == RobotStatus.ERROR:
        log_level = LogLevel.ERROR
        log_message = f"쿡봇 #{cookbot_in.robot_id}에 오류가 발생했습니다."
    else:
        log_message = f"쿡봇 #{cookbot_in.robot_id}의 상태가 '{cookbot_in.status}'로 변경되었습니다."
    
    # 상태 변경 시 추가 텍스트
    if status_changed:
        log_message += f" (이전 상태: {prev_status})"
    
    log_info(
        db,
        log_message,
        background_tasks
    )

    # 모듈 초기화 이후 동적 import
    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "cookbot",
        None
    )
    # 시스템 로그 브로드캐스트 예약
    background_tasks.add_task(
        broadcast_entity_update,
        "systemlog",
        None
    )

    return CookbotStatusResponse(
        robot_id=int(new_cookbot.robot_id),
        status=new_cookbot.status,
        timestamp=new_cookbot.timestamp
    )

@router.post("/{robot_id}/command", response_model=dict)
def send_command_to_cookbot(
    robot_id: int,
    command: dict,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Find robot
    robot = db.get(Robot, robot_id)
    if not robot:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND, 
            detail=f"Robot with ID {robot_id} not found"
        )
    
    if robot.robot_type != EntityType.COOKBOT:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Robot with ID {robot_id} is not a COOKBOT"
        )
    
    command_type = command.get("command_type")
    if not command_type:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Command type is required"
        )
    
    # 명령 처리 로직
    old_status = robot.status
    if command_type == "COOK":
        robot.status = RobotStatus.COOKING
        
        # Order processing logic
        order_id = command.get("order_id")
        if order_id:
            order = db.get(Order, order_id)
            if order:
                order.status = OrderStatus.COOKING
                db.add(order)
    elif command_type == "READY":
        robot.status = RobotStatus.READY
    else:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid command type: {command_type}"
        )
    
    robot.updated_at = datetime.utcnow()
    db.add(robot)
    db.commit()
    
    # Log command
    log_info(
        db,
        f"쿡봇 #{robot_id}에 명령 전송: {command_type}, 상태 변경: {old_status} → {robot.status}",
        background_tasks
    )
    
    return {
        "status": "success",
        "message": f"Command {command_type} sent to Cookbot {robot_id}"
    }
