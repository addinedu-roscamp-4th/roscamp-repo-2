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
    max_customer: int
    status: TableStatus
    x: float = None
    y: float = None
    width: float = None
    height: float = None

class TableCreateRequest(BaseModel):
    table_id: int
    max_customer: int
    x: float = None
    y: float = None
    width: float = None
    height: float = None

class AssignTableRequest(BaseModel):
    customer_id: int

# --- Router Endpoints ---
@router.get("", response_model=List[TableResponse])
def get_tables(db: Session = Depends(get_db)):
    tables = db.query(Table).all()
    return [
        TableResponse(
            id=t.id,
            max_customer=t.max_customer,
            status=t.status,
            x=t.x,
            y=t.y,
            width=t.width,
            height=t.height
        ) for t in tables
    ]

@router.post("", response_model=TableResponse)
def create_table(
    table_data: TableCreateRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Check if table number already exists
    existing = db.query(Table).filter(Table.id == table_data.id).first()
    if existing:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Table number {table_data.id} already exists"
        )
    # Create new table
    new_table = Table(
        table_id=table_data.table_id,
        max_customer=table_data.max_customer,
        status=TableStatus.AVAILABLE,
        x=table_data.x,
        y=table_data.y,
        width=table_data.width,
        height=table_data.height
    )
    db.add(new_table)
    db.commit()
    db.refresh(new_table)
    
    # 시스템 로그 생성
    log = SystemLog(
        level=LogLevel.INFO,
        message=f"새 테이블이 생성되었습니다. 테이블 번호: {new_table.id}, 최대 수용 인원: {new_table.max_customer}명",
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
        max_customer=new_table.max_customer,
        status=new_table.status,
        x=new_table.x,
        y=new_table.y,
        width=new_table.width,
        height=new_table.height
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
        timestamp=datetime.utcnow(),
        released_at=None
    )
    table.status = TableStatus.OCCUPIED
    db.add(assignment)
    db.add(table)
    db.commit()
    db.refresh(assignment)  # ID를 얻기 위해 리프레시
    
    # 시스템 로그 생성
    log = SystemLog(
        level=LogLevel.INFO,
        message=f"테이블이 할당되었습니다. 테이블 ID: {table.id}, 고객 ID: {assign_data.customer_id}, 인원 수: {customer.count}명",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # websocket broadcast
    from run import broadcast_entity_update
    background_tasks.add_task(broadcast_entity_update, "customer", None)
    background_tasks.add_task(broadcast_entity_update, "table", None)
    # 시스템 로그 브로드캐스트 예약
    background_tasks.add_task(broadcast_entity_update, "systemlog", None)
    
    # 클라이언트에 배정 ID를 전달하기 위한 응답 강화
    return TableResponse(
        id=table.id,
        max_customer=table.max_customer,
        status=table.status,
        x=table.x,
        y=table.y,
        width=table.width,
        height=table.height
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
    
    # 배정 정보를 삭제하는 대신 released_at 필드 업데이트
    assignment = db.query(GroupAssignment).filter(
        GroupAssignment.table_id == table_id,
        GroupAssignment.released_at == None
    ).first()
    
    if assignment:
        # 배정 삭제 대신 released_at 업데이트
        assignment.released_at = datetime.utcnow()
        db.add(assignment)
    
    # 테이블 상태 변경
    table.status = TableStatus.AVAILABLE
    db.add(table)
    db.commit()
    
    # 시스템 로그 생성
    log = SystemLog(
        level=LogLevel.INFO,
        message=f"테이블이 사용가능하게 되었습니다. 테이블 ID: {table.id}",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    db.refresh(log)
    
    # websocket broadcast
    from run import broadcast_entity_update
    background_tasks.add_task(broadcast_entity_update, "customer", None)
    background_tasks.add_task(broadcast_entity_update, "table", None)
    # 시스템 로그 브로드캐스트 예약
    background_tasks.add_task(broadcast_entity_update, "systemlog", None)
    
    return TableResponse(
        id=table.id,
        max_customer=table.max_customer,
        status=table.status,
        x=table.x,
        y=table.y,
        width=table.width,
        height=table.height
    )

class TableAssignmentResponse(BaseModel):
    id: int
    table_id: int
    customer_id: int
    timestamp: datetime

@router.get("/assignments", response_model=List[TableAssignmentResponse])
def get_table_assignments(db: Session = Depends(get_db)):
    """
    현재 테이블 배정 상태 조회
    """
    # JOIN을 사용하여 GroupAssignment와 Table 데이터를 함께 조회
    results = db.query(
        GroupAssignment.id, 
        GroupAssignment.table_id,
        GroupAssignment.customer_id,
        GroupAssignment.timestamp
    ).join(
        Table, GroupAssignment.table_id == Table.id
    ).filter(
        GroupAssignment.released_at == None
    ).all()
    
    return [
        TableAssignmentResponse(
            id=r.id,
            table_id=r.table_id,
            customer_id=r.customer_id,
            timestamp=r.timestamp
        )
        for r in results
    ]
