from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Event, SystemLog, User, AdminSettings, Notification
from app.models.enums import EventType, UserRole, EntityType, LogLevel, NotificationStatus
from app.routes.auth import get_current_user

router = APIRouter()

# --- Event Models ---
class EventResponse(BaseModel):
    id: int
    type: EventType
    related_entity_type: EntityType
    related_entity_id: int
    description: str
    timestamp: datetime

class EventCreateRequest(BaseModel):
    type: EventType
    related_entity_type: EntityType
    related_entity_id: int
    description: str

class SystemLogResponse(BaseModel):
    id: int
    level: str
    message: str
    timestamp: datetime

class SystemLogCreateRequest(BaseModel):
    level: str
    message: str

# --- Router Endpoints ---

@router.get("", response_model=List[EventResponse])
def get_events(db: Session = Depends(get_db)):
    events = db.query(Event).all()
    return [
        EventResponse(
            id=event.id,
            type=event.type,
            related_entity_type=event.related_entity_type,
            related_entity_id=event.related_entity_id,
            description=event.description,
            timestamp=event.timestamp
        ) for event in events
    ]

@router.get("/{event_id}", response_model=EventResponse)
def get_event(event_id: int, db: Session = Depends(get_db)):
    event = db.query(Event).filter(Event.id == event_id).first()
    if not event:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Event with ID {event_id} not found"
        )
    
    return EventResponse(
        id=event.id,
        type=event.type,
        related_entity_type=event.related_entity_type,
        related_entity_id=event.related_entity_id,
        description=event.description,
        timestamp=event.timestamp
    )

@router.post("", response_model=dict)
def create_event(event_data: EventCreateRequest, db: Session = Depends(get_db)):
    # Create new event
    new_event = Event(
        type=event_data.type,
        related_entity_type=event_data.related_entity_type,
        related_entity_id=event_data.related_entity_id,
        description=event_data.description,
        timestamp=datetime.utcnow()
    )
    
    db.add(new_event)
    db.commit()
    db.refresh(new_event)
    
    # Log this action
    log = SystemLog(
        level="INFO",
        message=f"새 이벤트 생성: {event_data.type}, 대상: {event_data.related_entity_type}/{event_data.related_entity_id}",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    
    return {
        "id": new_event.id,
        "status": "success",
        "message": "이벤트가 생성되었습니다."
    }

@router.delete("/{event_id}", response_model=dict)
def delete_event(
    event_id: int,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can delete events
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to delete events"
        )
    
    # Find event
    event = db.query(Event).filter(Event.id == event_id).first()
    if not event:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Event with ID {event_id} not found"
        )
    
    # Log event details before deleting
    event_type = event.type
    related_entity = f"{event.related_entity_type}/{event.related_entity_id}"
    
    # Delete event
    db.delete(event)
    db.commit()
    
    # Log this action
    log = SystemLog(
        level="INFO",
        message=f"이벤트 삭제: ID {event_id}, 유형: {event_type}, 대상: {related_entity}",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    
    return {"message": "이벤트가 삭제되었습니다."}

# --- System Logs Endpoints ---
@router.get("/system-logs", response_model=List[SystemLogResponse])
def get_system_logs(
    level: Optional[str] = None,
    limit: Optional[int] = 100,
    db: Session = Depends(get_db)
):
    # 시스템 로그를 누구나 조회 가능하도록 인증 요구사항 제거
    
    # Build query
    query = db.query(SystemLog)
    
    # Filter by level if provided
    if level:
        query = query.filter(SystemLog.level == level.upper())
    
    # Apply limit
    query = query.order_by(SystemLog.timestamp.desc()).limit(limit)
    
    logs = query.all()
    
    return [
        SystemLogResponse(
            id=log.id,
            level=log.level,
            message=log.message,
            timestamp=log.timestamp
        ) for log in logs
    ]

@router.post("/system-logs", response_model=dict)
def create_system_log(
    log_data: SystemLogCreateRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # 로그 생성도 인증 없이 가능하도록 변경
    
    # 로그 레벨 대문자로 정규화
    level = log_data.level.upper()
    
    # 로그 레벨 검증
    valid_levels = ["INFO", "WARNING", "ERROR", "DEBUG"]
    if level not in valid_levels:
        level = "INFO"  # 기본값
    
    # Create new log
    new_log = SystemLog(
        level=level,
        message=log_data.message,
        timestamp=datetime.utcnow()
    )
    
    db.add(new_log)
    db.commit()
    db.refresh(new_log)
    
    # 알림 생성 (백그라운드 작업으로 처리)
    background_tasks.add_task(create_notifications_from_log, new_log.id, db)
    
    # 브로드캐스팅 작업 추가
    from run import broadcast_entity_update
    background_tasks.add_task(
        broadcast_entity_update,
        "systemlog",
        None
    )
    
    return {
        "id": new_log.id,
        "status": "success",
        "message": "로그가 생성되었습니다."
    }

# --- 알림 생성 함수 ---
def create_notifications_from_log(log_id: int, db: Session):
    """시스템 로그에서 사용자 알림을 생성"""


    # 로그 가져오기
    log = db.query(SystemLog).filter(SystemLog.id == log_id).first()
    if not log:
        return
    
    # 알림 설정 가져오기
    settings = db.query(AdminSettings).first()
    if not settings or not settings.alert_settings:
        return
    
    # alert_settings = {"push_notifications": true, "INFO": true, "WARNING": true, "ERROR": true, "DEBUG": true}
    # 로그 레벨에 따른 알림 설정 확인
    alert_settings = settings.alert_settings
    alert_settings = {
        "INFO": alert_settings.get("INFO", False),
        "WARNING": alert_settings.get("WARNING", False),
        "ERROR": alert_settings.get("ERROR", False),
        "DEBUG": alert_settings.get("DEBUG", False)
    }
    if not alert_settings.get(log.level, False):
        return

    
    # 알림을 받을 사용자 목록 가져오기 (현재는 모든 ADMIN 사용자)
    users = db.query(User).filter(User.role == UserRole.ADMIN).all()
    
    # 각 사용자에 대해 알림 생성
    notifications = []
    for user in users:
        notification = Notification(
            user_id=user.id,
            type=log.level,
            message=log.message,
            created_at=datetime.utcnow(),
            status=NotificationStatus.PENDING
        )
        notifications.append(notification)
    
    # 모든 알림을 한 번에 추가하고 커밋
    if notifications:
        db.add_all(notifications)
        db.commit()
        
        # 브로드캐스팅을 위한 broadcast_notifications_update 호출
        from run import broadcast_notifications_update
        broadcast_notifications_update(None)

# SystemLog 로그 레벨별 유틸리티 함수들
def log_info(db: Session, message: str, broadcast_tasks=None):
    """INFO 레벨 로그 생성 및 옵션으로 브로드캐스팅"""
    log = SystemLog(level=LogLevel.INFO, message=message, timestamp=datetime.utcnow())
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # 알림 생성
    create_notifications_from_log(log.id, db)
    
    # 브로드캐스트 작업이 제공된 경우 예약
    if broadcast_tasks:
        from run import broadcast_entity_update
        broadcast_tasks.add_task(
            broadcast_entity_update,
            "systemlog",
            None
        )
    return log

def log_warning(db: Session, message: str, broadcast_tasks=None):
    """WARNING 레벨 로그 생성 및 옵션으로 브로드캐스팅"""
    log = SystemLog(level=LogLevel.WARNING, message=message, timestamp=datetime.utcnow())
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # 알림 생성
    create_notifications_from_log(log.id, db)
    
    # 브로드캐스트 작업이 제공된 경우 예약
    if broadcast_tasks:
        from run import broadcast_entity_update
        broadcast_tasks.add_task(
            broadcast_entity_update,
            "systemlog",
            None
        )
    return log

def log_error(db: Session, message: str, broadcast_tasks=None):
    """ERROR 레벨 로그 생성 및 옵션으로 브로드캐스팅"""
    log = SystemLog(level=LogLevel.ERROR, message=message, timestamp=datetime.utcnow())
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # 알림 생성
    create_notifications_from_log(log.id, db)
    
    # 브로드캐스트 작업이 제공된 경우 예약
    if broadcast_tasks:
        from run import broadcast_entity_update
        broadcast_tasks.add_task(
            broadcast_entity_update,
            "systemlog",
            None
        )
    return log

def log_debug(db: Session, message: str, broadcast_tasks=None):
    """DEBUG 레벨 로그 생성 및 옵션으로 브로드캐스팅"""
    log = SystemLog(level=LogLevel.DEBUG, message=message, timestamp=datetime.utcnow())
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # 알림 생성
    create_notifications_from_log(log.id, db)
    
    # 브로드캐스트 작업이 제공된 경우 예약
    if broadcast_tasks:
        from run import broadcast_entity_update
        broadcast_tasks.add_task(
            broadcast_entity_update,
            "systemlog",
            None
        )
    return log

def create_system_log(db: Session, level: LogLevel, message: str, broadcast_tasks=None):
    """지정된 레벨로 시스템 로그 생성 및 옵션으로 브로드캐스팅"""
    log = SystemLog(level=level, message=message, timestamp=datetime.utcnow())
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # 알림 생성
    create_notifications_from_log(log.id, db)
    
    # 브로드캐스트 작업이 제공된 경우 예약
    if broadcast_tasks:
        from run import broadcast_entity_update
        broadcast_tasks.add_task(
            broadcast_entity_update,
            "systemlog",
            None
        )
    return log 