from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Table, GroupAssignment, Customer
from app.models.enums import TableStatus

router = APIRouter()

# --- Table Models ---
class TableResponse(BaseModel):
    id: int
    table_number: int
    max_customer: int
    status: TableStatus

class TableCreateRequest(BaseModel):
    number: int
    max_customer: int

class AssignTableRequest(BaseModel):
    customer_id: int

# --- Router Endpoints ---
@router.get("", response_model=List[TableResponse])
def get_tables(db: Session = Depends(get_db)):
    tables = db.query(Table).all()
    return [
        TableResponse(
            id=table.id,
            table_number=table.table_number,
            max_customer=table.max_customer,
            status=table.status
        ) for table in tables
    ]

@router.post("", response_model=dict)
def create_table(table_data: TableCreateRequest, db: Session = Depends(get_db)):
    # Check if table number already exists
    existing_table = db.query(Table).filter(Table.table_number == table_data.number).first()
    if existing_table:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Table number {table_data.number} already exists"
        )
    
    # Create new table
    new_table = Table(
        table_number=table_data.number,
        max_customer=table_data.max_customer,
        status=TableStatus.AVAILABLE
    )
    
    db.add(new_table)
    db.commit()
    db.refresh(new_table)
    
    return {"id": new_table.id, "status": "success"}

@router.post("/{table_id}/assign")
def assign_table(table_id: int, assign_data: AssignTableRequest, db: Session = Depends(get_db)):
    # Find table
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
            detail="Table is not available for assignment"
        )
    
    # Find customer
    customer = db.query(Customer).filter(Customer.id == assign_data.customer_id).first()
    if not customer:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Customer with ID {assign_data.customer_id} not found"
        )
    
    # Check if customer count is within table capacity
    if customer.count > table.max_customer:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Customer group size ({customer.count}) exceeds table capacity ({table.max_customer})"
        )
    
    # Create assignment
    assignment = GroupAssignment(
        table_id=table_id,
        customer_id=assign_data.customer_id,
        assigned_at=datetime.utcnow()
    )
    
    # Update table status
    table.status = TableStatus.OCCUPIED
    
    db.add(assignment)
    db.add(table)
    db.commit()
    
    return {"status": "success", "message": "테이블에 배정되었습니다."}

@router.post("/{table_id}/release")
def release_table(table_id: int, db: Session = Depends(get_db)):
    # Find table
    table = db.query(Table).filter(Table.id == table_id).first()
    if not table:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Table with ID {table_id} not found"
        )
    
    # Check if table is occupied
    if table.status != TableStatus.OCCUPIED:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Table is not currently occupied"
        )
    
    # Find and remove assignment
    assignment = db.query(GroupAssignment).filter(GroupAssignment.table_id == table_id).first()
    if assignment:
        db.delete(assignment)
    
    # Update table status
    table.status = TableStatus.AVAILABLE
    
    db.add(table)
    db.commit()
    
    return {"status": "success", "message": "테이블 배정이 해제되었습니다."} 