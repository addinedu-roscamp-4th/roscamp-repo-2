from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import List
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Table, GroupAssignment, Customer
from app.models.enums import TableStatus, LogLevel
from app.models.event import SystemLog

router = APIRouter()

# --- Table Models ---
class TableResponse(BaseModel):
    id: int
    table_number: int
    max_customer: int
    status: TableStatus

class TableCreateRequest(BaseModel):
    table_number: int
    max_customer: int

class AssignTableRequest(BaseModel):
    customer_id: int

# --- Router Endpoints ---
@router.get("", response_model=List[TableResponse])
def get_tables(db: Session = Depends(get_db)):
    tables = db.query(Table).all()
    return [
        TableResponse(
            id=t.id,
            table_number=t.table_number,
            max_customer=t.max_customer,
            status=t.status
        ) for t in tables
    ]

@router.post("", response_model=TableResponse)
def create_table(
    table_data: TableCreateRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Check if table number already exists
    existing = db.query(Table).filter(Table.table_number == table_data.table_number).first()
    if existing:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Table number {table_data.table_number} already exists"
        )
    # Create new table
    new_table = Table(
        table_number=table_data.table_number,
        max_customer=table_data.max_customer,
        status=TableStatus.AVAILABLE
    )
    db.add(new_table)
    db.commit()
    db.refresh(new_table)
    
    # 시스템 로그 생성
    log = SystemLog(
        level=LogLevel.INFO,
        message=f"새 테이블이 생성되었습니다. 테이블 번호: {new_table.table_number}, 최대 수용 인원: {new_table.max_customer}명",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # trigger websocket broadcast
    from run import broadcast_entity_update
    background_tasks.add_task(broadcast_entity_update, "table", None)
    # 시스템 로그 브로드캐스트 예약
    background_tasks.add_task(broadcast_entity_update, "systemlog", None)
    
    return TableResponse(
        id=new_table.id,
        table_number=new_table.table_number,
        max_customer=new_table.max_customer,
        status=new_table.status
    )

@router.post("/{table_id}/assign", response_model=TableResponse)
def assign_table(
    table_id: int,
    assign_data: AssignTableRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Find table
    table = db.query(Table).filter(Table.id == table_id).first()
    if not table:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"Table ID {table_id} not found")
    if table.status != TableStatus.AVAILABLE:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Table not available")
    # Find customer
    customer = db.query(Customer).filter(Customer.id == assign_data.customer_id).first()
    if not customer:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"Customer ID {assign_data.customer_id} not found")
    if customer.count > table.max_customer:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Group size {customer.count} exceeds capacity {table.max_customer}")
    # Assignment
    assignment = GroupAssignment(
        table_id=table_id,
        customer_id=assign_data.customer_id,
        assigned_at=datetime.utcnow()
    )
    table.status = TableStatus.OCCUPIED
    db.add(assignment)
    db.add(table)
    db.commit()
    
    # 시스템 로그 생성
    log = SystemLog(
        level=LogLevel.INFO,
        message=f"테이블이 할당되었습니다. 테이블 번호: {table.table_number}, 고객 ID: {assign_data.customer_id}, 인원 수: {customer.count}명",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # websocket broadcast
    from run import broadcast_entity_update
    background_tasks.add_task(broadcast_entity_update, "table", None)
    # 시스템 로그 브로드캐스트 예약
    background_tasks.add_task(broadcast_entity_update, "systemlog", None)
    
    return TableResponse(
        id=table.id,
        table_number=table.table_number,
        max_customer=table.max_customer,
        status=table.status
    )

@router.post("/{table_id}/release", response_model=TableResponse)
def release_table(
    table_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    table = db.query(Table).filter(Table.id == table_id).first()
    if not table:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"Table ID {table_id} not found")
    if table.status != TableStatus.OCCUPIED:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Table not occupied")
    assignment = db.query(GroupAssignment).filter(GroupAssignment.table_id == table_id).first()
    if assignment:
        db.delete(assignment)
    table.status = TableStatus.AVAILABLE
    db.add(table)
    db.commit()
    
    # 시스템 로그 생성
    log = SystemLog(
        level=LogLevel.INFO,
        message=f"테이블이 사용가능하게 되었습니다. 테이블 번호: {table.table_number}",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # websocket broadcast
    from run import broadcast_entity_update
    background_tasks.add_task(broadcast_entity_update, "table", None)
    # 시스템 로그 브로드캐스트 예약
    background_tasks.add_task(broadcast_entity_update, "systemlog", None)
    
    return TableResponse(
        id=table.id,
        table_number=table.table_number,
        max_customer=table.max_customer,
        status=table.status
    )
