# app/routes/customers.py
from fastapi import APIRouter, Depends, status, BackgroundTasks
from fastapi import HTTPException
from sqlalchemy.orm import Session
from typing import List
from pydantic import BaseModel
from datetime import datetime

from app.core.db_config import get_db
from app.models.customer import Customer
from app.models.event import SystemLog
from app.models.enums import LogLevel

router = APIRouter(
    prefix="/customers",
    tags=["customers"],
)

# --- Pydantic Schemas ---
class CustomerCreateRequest(BaseModel):
    count: int

class CustomerResponse(BaseModel):
    id: int
    count: int
    timestamp: datetime

    class Config:
        orm_mode = True

# --- Endpoints ---
@router.get("", response_model=List[CustomerResponse])
def list_customers(db: Session = Depends(get_db)):
    """
    모든 고객 그룹 조회
    """
    return db.query(Customer).all()

@router.post("", response_model=CustomerResponse, status_code=status.HTTP_201_CREATED)
def create_customer(request: CustomerCreateRequest, 
                     background_tasks: BackgroundTasks,
                     db: Session = Depends(get_db)):
    """
    새로운 고객 그룹 생성
    """
    new_customer = Customer(count=request.count)
    db.add(new_customer)
    db.commit()
    db.refresh(new_customer)
    
    # 시스템 로그 생성
    log = SystemLog(
        level=LogLevel.INFO,
        message=f"새로운 손님이 입장했습니다. 인원 수: {request.count}명, 고객 ID: {new_customer.id}",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # 시스템 로그 브로드캐스트
    from run import broadcast_entity_update
    background_tasks.add_task(
        broadcast_entity_update,
        "systemlog",
        None
    )
    
    return new_customer
