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

router = APIRouter()

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
    customers = (
    db.query(Customer)
    .order_by(Customer.id.desc())  # 최신 고객 그룹이 위에 오도록 정렬
    .all()
    )
    
    return [
        CustomerResponse(
            id=customer.id,
            count=customer.count,
            timestamp=customer.timestamp
        )
        for customer in customers
    ]

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
    background_tasks.add_task(
        broadcast_entity_update,
        "customer",
        None
    )
    
    return new_customer

@router.delete("/{customer_id}", status_code=status.HTTP_204_NO_CONTENT)
def delete_customer(customer_id: int,
                        background_tasks: BackgroundTasks,
                     db: Session = Depends(get_db)):
    """
    고객 그룹 삭제
    """
    customer = db.query(Customer).filter(Customer.id == customer_id).first()
    
    if not customer:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Customer with ID {customer_id} not found"
        )
    
    db.delete(customer)
    db.commit()
    
    # 시스템 로그 생성
    log = SystemLog(
        level=LogLevel.INFO,
        message=f"손님이 퇴장했습니다. 고객 ID: {customer_id}",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()

    from run import broadcast_entity_update
    background_tasks.add_task(
        broadcast_entity_update,
        "customer",
        None
    )
    
    return {"message": "Customer deleted successfully"}
