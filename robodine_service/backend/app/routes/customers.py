from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Customer, User
from app.routes.auth import get_current_user

router = APIRouter()

# --- Customer Models ---
class CustomerResponse(BaseModel):
    id: int
    count: int
    created_at: datetime

class CustomerCreateRequest(BaseModel):
    count: int

# --- Router Endpoints ---
@router.get("", response_model=List[CustomerResponse])
def get_customers(db: Session = Depends(get_db)):
    customers = db.query(Customer).all()
    return [
        CustomerResponse(
            id=customer.id,
            count=customer.count,
            created_at=customer.created_at
        ) for customer in customers
    ]

@router.post("", response_model=CustomerResponse)
def create_customer(customer_data: CustomerCreateRequest, db: Session = Depends(get_db)):
    # Create new customer group
    new_customer = Customer(
        count=customer_data.count,
        created_at=datetime.utcnow()
    )
    
    db.add(new_customer)
    db.commit()
    db.refresh(new_customer)
    
    return CustomerResponse(
        id=new_customer.id,
        count=new_customer.count,
        created_at=new_customer.created_at
    ) 