from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import List
from datetime import datetime

from app.core.db_config import get_db
from app.models import User, Notification
from app.models.enums import UserRole, NotificationStatus
from app.routes.auth import get_current_user, get_password_hash
from pydantic import BaseModel

router = APIRouter(prefix="/api/users")


# --- User DTOs ---
from pydantic import BaseModel, validator

class UserResponse(BaseModel):
    id: int
    username: str
    name: str
    role: UserRole
    created_at: datetime
    updated_at: datetime
    last_login: datetime | None

    @validator("name", pre=True, always=True)
    def default_name(cls, v):
        return v or ""

    class Config:
        orm_mode = True


class UserListResponse(BaseModel):
    id: int
    username: str
    name: str
    role: UserRole
    created_at: datetime
    updated_at: datetime
    last_login: datetime | None

    @validator("name", pre=True, always=True)
    def default_name(cls, v):
        return v or ""

    class Config:
        orm_mode = True

class UserCreateRequest(BaseModel):
    username: str
    password: str
    name: str
    role: UserRole = UserRole.ADMIN


class UserUpdateRequest(BaseModel):
    name: str | None = None
    role: UserRole | None = None
    password: str | None = None


# --- Notification DTOs ---
class NotificationResponse(BaseModel):
    id: int
    type: str | None
    message: str | None
    created_at: datetime
    status: NotificationStatus | None

    class Config:
        orm_mode = True


class NotificationCreateRequest(BaseModel):
    type: str
    message: str
    
class NotificationUpdateRequest(BaseModel):
    status: NotificationStatus


# --- Endpoints ---
@router.get("/me", response_model=UserResponse)
def read_current_user(current_user: User = Depends(get_current_user)):
    return current_user


@router.get("", response_model=List[UserListResponse])
def read_users(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to view all users"
        )
    return db.query(User).all()


@router.get("/{user_id}", response_model=UserResponse)
def read_user(
    user_id: int,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    if current_user.role != UserRole.ADMIN and current_user.id != user_id:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN,
                            detail="Not authorized to access this user")
    user = db.get(User, user_id)
    if not user:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                            detail="User not found")
    return user


@router.post("", status_code=status.HTTP_201_CREATED, response_model=UserResponse)
def create_user(
    payload: UserCreateRequest,
    db: Session = Depends(get_db)
):
    if db.query(User).filter(User.username == payload.username).first():
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST,
                            detail="Username already exists")
    user = User(
        username=payload.username,
        password_hash=get_password_hash(payload.password),
        name=payload.name,
        role=payload.role
    )
    db.add(user)
    db.commit()
    db.refresh(user)
    return user


@router.put("/{user_id}", response_model=UserResponse)
def update_user(
    user_id: int,
    payload: UserUpdateRequest,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    if current_user.role != UserRole.ADMIN and current_user.id != user_id:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN,
                            detail="Not authorized to update this user")
    user = db.get(User, user_id)
    if not user:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                            detail="User not found")

    if payload.name is not None:
        user.name = payload.name
    if payload.role is not None and current_user.role == UserRole.ADMIN:
        user.role = payload.role
    if payload.password is not None:
        user.password_hash = get_password_hash(payload.password)

    user.updated_at = datetime.utcnow()
    db.add(user)
    db.commit()
    db.refresh(user)
    return user


@router.delete("/{user_id}", status_code=status.HTTP_204_NO_CONTENT)
def delete_user(
    user_id: int,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN,
                            detail="Not authorized to delete users")
    user = db.get(User, user_id)
    if not user:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                            detail="User not found")
    db.delete(user)
    db.commit()


# --- Notifications ---
@router.get("/{user_id}/notifications", response_model=List[NotificationResponse])
def read_notifications(
    user_id: int,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    if current_user.role != UserRole.ADMIN and current_user.id != user_id:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN,
                            detail="Not authorized to view these notifications")
    if not db.get(User, user_id):
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                            detail="User not found")
    return db.query(Notification).filter(Notification.user_id == user_id).all()


@router.post(
    "/{user_id}/notifications",
    status_code=status.HTTP_201_CREATED,
    response_model=NotificationResponse
)
def create_notification(
    user_id: int,
    payload: NotificationCreateRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    if not db.get(User, user_id):
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                            detail="User not found")
    note = Notification(
        user_id=user_id,
        type=payload.type,
        message=payload.message,
        status=NotificationStatus.PENDING
    )
    db.add(note)
    db.commit()
    db.refresh(note)
    
    # 알림 생성 후 웹소켓 업데이트 예약
    from run import broadcast_entity_update
    background_tasks.add_task(
        broadcast_entity_update,
        "notification",
        None
    )
    
    return note

@router.put(
    "/{user_id}/notifications/{notification_id}",
    response_model=NotificationResponse
)
def update_notification_status(
    user_id: int,
    notification_id: int,
    payload: NotificationUpdateRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # 자신의 알림만 업데이트 가능
    if current_user.role != UserRole.ADMIN and current_user.id != user_id:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN,
                           detail="Not authorized to update this notification")
    
    # 사용자 존재 확인
    if not db.get(User, user_id):
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                           detail="User not found")
    
    # 알림 존재 확인
    notification = db.query(Notification).filter(
        Notification.id == notification_id,
        Notification.user_id == user_id
    ).first()
    
    if not notification:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                           detail="Notification not found")
    
    # 알림 상태 업데이트
    notification.status = payload.status
    db.add(notification)
    db.commit()
    db.refresh(notification)
    
    # 웹소켓 업데이트 예약
    from run import broadcast_entity_update
    background_tasks.add_task(
        broadcast_entity_update,
        "notification",
        None
    )
    
    return notification
