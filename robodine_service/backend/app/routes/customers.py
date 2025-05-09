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
from app.routes.events import log_info, log_warning, log_error
from app.models.table import Table, GroupAssignment
from app.models.enums import TableStatus


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

@router.post("", response_model=dict)
def create_customer(
    customer_data: CustomerCreateRequest, 
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    
    # Create new customer
    customer = Customer(
        count=customer_data.count,
        timestamp=datetime.utcnow()
    )
    db.add(customer)
    db.commit()
    db.refresh(customer)
    
    # Log this action
    log_info(db, f"손님이 입장하셨습니다.: {customer.count}명 (ID: {customer.id})", background_tasks)
    
    return {"id": customer.id, "message": "Customer created successfully"}

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
    
    # Log this action
    log_info(db, f"손님이 퇴장했습니다. 고객 ID: {customer_id}", background_tasks)
    
    return {"message": "Customer deleted successfully"}

@router.put("/{customer_id}/assign-table/{table_id}", response_model=dict)
def assign_table_to_customer(
    customer_id: int, 
    table_id: int, 
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Check if customer exists
    customer = db.query(Customer).filter(Customer.id == customer_id).first()
    if not customer:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Customer with ID {customer_id} not found"
        )
    
    # Check if table exists
    table = db.query(Table).filter(Table.id == table_id).first()
    if not table:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Table with ID {table_id} not found"
        )
    
    # Check if table is available
    if table.status != TableStatus.AVAILABLE:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Table {table_id} is not available (current status: {table.status})"
        )
    
    # Check if table can accommodate the customer group
    if customer.count > table.max_customer:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Table {table_id} cannot accommodate {customer.count} customers (max: {table.max_customer})"
        )
    
    # Create assignment
    assignment = GroupAssignment(
        table_id=table_id,
        customer_id=customer_id,
        timestamp=datetime.utcnow()
    )
    
    # Update table status
    table.status = TableStatus.OCCUPIED
    table.updated_at = datetime.utcnow()
    
    # Add both changes to DB
    db.add(assignment)
    db.add(table)
    db.commit()
    
    # Log this action
    log_info(db, f"테이블 {table_id} 배정되었습니다: 고객 그룹 {customer_id} ({customer.count}명)", background_tasks)
    
    return {"message": f"Customer {customer_id} assigned to table {table_id}"}
