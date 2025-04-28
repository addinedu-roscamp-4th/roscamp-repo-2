from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import User, Notification
from app.models.enums import UserRole, NotificationStatus
from app.routes.auth import get_current_user, get_password_hash

router = APIRouter()

# --- User Models ---
class UserResponse(BaseModel):
    id: int
    username: str
    role: UserRole
    created_at: datetime
    updated_at: datetime

class UserListResponse(BaseModel):
    id: int
    username: str
    role: UserRole
    created_at: datetime
    updated_at: datetime

class UserCreateRequest(BaseModel):
    username: str
    password: str
    role: UserRole

class UserUpdateRequest(BaseModel):
    password: Optional[str] = None
    role: Optional[UserRole] = None

class NotificationResponse(BaseModel):
    id: int
    type: str
    message: str
    created_at: datetime
    status: NotificationStatus

class NotificationCreateRequest(BaseModel):
    type: str
    message: str

# --- Router Endpoints ---
@router.get("", response_model=List[UserListResponse])
def get_users(db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    # Only admin users can list all users
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to view all users"
        )
    
    users = db.query(User).all()
    return [
        UserListResponse(
            id=user.id,
            username=user.username,
            role=user.role,
            created_at=user.created_at,
            updated_at=user.updated_at
        ) for user in users
    ]

@router.get("/{user_id}", response_model=UserResponse)
def get_user(user_id: int, db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    # Users can view their own info, admins can view anyone
    if current_user.id != user_id and current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to view this user"
        )
    
    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"User with ID {user_id} not found"
        )
    
    return UserResponse(
        id=user.id,
        username=user.username,
        role=user.role,
        created_at=user.created_at,
        updated_at=user.updated_at
    )

@router.post("", response_model=UserResponse)
def create_user(user_data: UserCreateRequest, db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    # Only admins can create users
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to create users"
        )
    
    # Check if username already exists
    existing_user = db.query(User).filter(User.username == user_data.username).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Username {user_data.username} already exists"
        )
    
    # Create new user
    new_user = User(
        username=user_data.username,
        password_hash=get_password_hash(user_data.password),
        role=user_data.role,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    
    db.add(new_user)
    db.commit()
    db.refresh(new_user)
    
    return UserResponse(
        id=new_user.id,
        username=new_user.username,
        role=new_user.role,
        created_at=new_user.created_at,
        updated_at=new_user.updated_at
    )

@router.put("/{user_id}")
def update_user(
    user_id: int,
    user_data: UserUpdateRequest, 
    db: Session = Depends(get_db), 
    current_user: User = Depends(get_current_user)
):
    # Users can update their own info, admins can update anyone
    if current_user.id != user_id and current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to update this user"
        )
    
    # Find user
    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"User with ID {user_id} not found"
        )
    
    # Update user fields
    if user_data.password:
        user.password_hash = get_password_hash(user_data.password)
    
    # Only admins can change roles
    if user_data.role and current_user.role == UserRole.ADMIN:
        user.role = user_data.role
    
    user.updated_at = datetime.utcnow()
    
    db.add(user)
    db.commit()
    
    return {"status": "success", "message": "사용자 정보가 성공적으로 수정되었습니다."}

# --- Notifications ---
@router.get("/{user_id}/notifications", response_model=List[NotificationResponse])
def get_notifications(
    user_id: int, 
    db: Session = Depends(get_db), 
    current_user: User = Depends(get_current_user)
):
    # Users can view their own notifications, admins can view anyone's
    if current_user.id != user_id and current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to view these notifications"
        )
    
    # Find user
    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"User with ID {user_id} not found"
        )
    
    # Get notifications
    notifications = db.query(Notification).filter(Notification.user_id == user_id).all()
    
    return [
        NotificationResponse(
            id=notification.id,
            type=notification.type,
            message=notification.message,
            created_at=notification.created_at,
            status=notification.status
        ) for notification in notifications
    ]

@router.post("/{user_id}/notifications")
def create_notification(
    user_id: int,
    notification_data: NotificationCreateRequest,
    db: Session = Depends(get_db)
):
    # Find user
    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"User with ID {user_id} not found"
        )
    
    # Create notification
    new_notification = Notification(
        user_id=user_id,
        type=notification_data.type,
        message=notification_data.message,
        created_at=datetime.utcnow(),
        status=NotificationStatus.PENDING
    )
    
    db.add(new_notification)
    db.commit()
    db.refresh(new_notification)
    
    return {
        "id": new_notification.id,
        "status": "success",
        "message": "알림이 생성되었습니다."
    } 