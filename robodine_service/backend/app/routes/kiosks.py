from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import KioskTerminal
from app.routes.auth import get_current_user, get_password_hash

router = APIRouter()

# --- Kiosk Models ---
class KioskResponse(BaseModel):
    id: int
    table_number: Optional[int]
    ip_address: str

class KioskCreateRequest(BaseModel):
    table_number: Optional[int]
    ip_address: str

# --- Router Endpoints ---
@router.get("", response_model=List[KioskResponse])
def get_kiosks(db: Session = Depends(get_db)):
    kiosks = db.query(KioskTerminal).all()
    return [
        KioskResponse(
            id=kiosk.id,
            table_number=kiosk.table_number,
            ip_address=kiosk.ip_address
        ) for kiosk in kiosks
    ]

@router.post("", response_model=dict)
def create_kiosk(kiosk_data: KioskCreateRequest, db: Session = Depends(get_db)):
    # Create new kiosk
    new_kiosk = KioskTerminal(
        table_number=kiosk_data.table_number,
        ip_address=kiosk_data.ip_address
    )
    
    db.add(new_kiosk)
    db.commit()
    db.refresh(new_kiosk)
    
    return {"id": new_kiosk.id, "status": "success"} 