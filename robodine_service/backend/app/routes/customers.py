# app/routes/customers.py
from fastapi import APIRouter, Depends, status
from fastapi import HTTPException
from sqlalchemy.orm import Session
from typing import List
from pydantic import BaseModel
from datetime import datetime

from app.core.db_config import get_db
from app.models.customer import Customer

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
def create_customer(request: CustomerCreateRequest, db: Session = Depends(get_db)):
    """
    새로운 고객 그룹 생성
    """
    new_customer = Customer(count=request.count)
    db.add(new_customer)
    db.commit()
    db.refresh(new_customer)
    return new_customer
