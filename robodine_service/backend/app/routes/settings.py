from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import AdminSettings, User
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

# --- Settings Models ---
class AdminSettingsResponse(BaseModel):
    id: Optional[int] = None
    store_name: Optional[str] = None
    operation_start: str
    operation_end: str
    inventory_threshold: int
    alert_settings: Dict

class AdminSettingsUpdateRequest(BaseModel):
    id: Optional[int] = None
    store_name: Optional[str] = None
    operation_start: Optional[str] = None
    operation_end: Optional[str] = None
    inventory_threshold: Optional[int] = None
    alert_settings: Optional[Dict] = None

# --- Router Endpoints ---
@router.get("", response_model=AdminSettingsResponse)
def get_settings(db: Session = Depends(get_db)):
    # Get settings from database
    settings = db.query(AdminSettings).first()
    
    # If no settings exist, create default settings
    if not settings:
        settings = AdminSettings(
            operation_start="08:00",
            operation_end="22:00",
            inventory_threshold=10,
            alert_settings={"low_stock": True}
        )
        db.add(settings)
        db.commit()
        db.refresh(settings)
    
    return AdminSettingsResponse(
        id=settings.id,
        store_name=settings.store_name,
        operation_start=settings.operation_start,
        operation_end=settings.operation_end,
        inventory_threshold=settings.inventory_threshold,
        alert_settings=settings.alert_settings
    )

@router.put("")
def update_settings(
    settings_data: AdminSettingsUpdateRequest,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can update settings
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to update settings"
        )
    
    # Get current settings
    settings = db.query(AdminSettings).first()
    
    # Update settings
    if settings_data.operation_start:
        settings.operation_start = settings_data.operation_start
    
    if settings_data.operation_end:
        settings.operation_end = settings_data.operation_end
    
    if settings_data.inventory_threshold is not None:
        settings.inventory_threshold = settings_data.inventory_threshold

    if settings_data.store_name:
        settings.store_name = settings_data.store_name
    
    if settings_data.alert_settings:
        settings.alert_settings = settings_data.alert_settings
    
    db.add(settings)
    db.commit()
    
    return {"status": "success", "message": "설정이 업데이트되었습니다."} 