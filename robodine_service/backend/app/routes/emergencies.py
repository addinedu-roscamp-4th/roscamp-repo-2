from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Emergency, User
from app.models.enums import UserRole
from app.routes.auth import get_current_user

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

# --- Emergency Models ---
class EmergencyResponse(BaseModel):
    id: int
    emergency_type: str
    description: str
    is_active: bool
    reported_at: datetime
    resolved_at: Optional[datetime] = None

class EmergencyCreateRequest(BaseModel):
    emergency_type: str
    description: str

# --- Router Endpoints ---
@router.get("", response_model=List[EmergencyResponse])
def get_emergencies(db: Session = Depends(get_db)):
    emergencies = db.query(Emergency).all()
    return [
        EmergencyResponse(
            id=emergency.id,
            emergency_type=emergency.emergency_type,
            description=emergency.description,
            is_active=emergency.is_active,
            reported_at=emergency.reported_at,
            resolved_at=emergency.resolved_at
        ) for emergency in emergencies
    ]

@router.post("", response_model=dict)
def create_emergency(emergency_data: EmergencyCreateRequest, db: Session = Depends(get_db)):
    # Create new emergency
    new_emergency = Emergency(
        emergency_type=emergency_data.emergency_type,
        description=emergency_data.description,
        is_active=True,
        reported_at=datetime.utcnow(),
        resolved_at=None
    )
    
    db.add(new_emergency)
    db.commit()
    db.refresh(new_emergency)
    
    return {
        "id": new_emergency.id,
        "status": "success",
        "message": "비상 상황이 신고되었습니다."
    }

@router.put("/{emergency_id}/resolve", response_model=dict)
def resolve_emergency(
    emergency_id: int,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can resolve emergencies
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to resolve emergencies"
        )
    
    # Find emergency
    emergency = db.query(Emergency).filter(Emergency.id == emergency_id).first()
    if not emergency:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Emergency with ID {emergency_id} not found"
        )
    
    # Check if already resolved
    if not emergency.is_active:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Emergency has already been resolved"
        )
    
    # Resolve emergency
    emergency.is_active = False
    emergency.resolved_at = datetime.utcnow()
    
    db.add(emergency)
    db.commit()
    
    return {"status": "success", "message": "비상 상황이 해제되었습니다."} 