from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import KioskTerminal
from app.routes.auth import get_current_user, get_password_hash

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

# --- Kiosk Models ---
class KioskResponse(BaseModel):
    id: int
    table_id: Optional[int]
    ip_address: str

class KioskCreateRequest(BaseModel):
    table_id: Optional[int]
    ip_address: str

# --- Router Endpoints ---
@router.get("", response_model=List[KioskResponse])
def get_kiosks(db: Session = Depends(get_db)):
    kiosks = db.query(KioskTerminal).all()
    return [
        KioskResponse(
            id=kiosk.id,
            table_id=kiosk.table_id,
            ip_address=kiosk.ip_address
        ) for kiosk in kiosks
    ]

@router.post("", response_model=dict)
def create_kiosk(kiosk_data: KioskCreateRequest, db: Session = Depends(get_db)):
    # Create new kiosk
    new_kiosk = KioskTerminal(
        table_id=kiosk_data.table_id,
        ip_address=kiosk_data.ip_address
    )
    
    db.add(new_kiosk)
    db.commit()
    db.refresh(new_kiosk)
    
    return {"id": new_kiosk.id, "status": "success"} 