from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import List
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Table, GroupAssignment, Customer
from app.models.enums import TableStatus, LogLevel
from app.routes.events import log_info, log_warning, log_error

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

@router.post("", response_model=dict)
def create_table(
    table_data: dict,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Create new table
    new_table = Table(
        max_customer=table_data.get("max_customer", 4),
        status=TableStatus.AVAILABLE,
        x=table_data.get("x", 0),
        y=table_data.get("y", 0),
        width=table_data.get("width", 100),
        height=table_data.get("height", 100),
        updated_at=datetime.utcnow()
    )
    
    db.add(new_table)
    db.commit()
    db.refresh(new_table)
    
    # Log this action
    log_info(db, f"새 테이블 생성: {new_table.id}, 최대 인원 {new_table.max_customer}명", background_tasks)
    
    return {
        "id": new_table.id,
        "status": "success",
        "message": "Table created successfully"
    }

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
    log_info(db, f"테이블 {table.id}에 고객 {assign_data.customer_id} 할당되었습니다", background_tasks)

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

@router.put("/{table_id}/release", response_model=dict)
def release_table(
    table_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Find table
    table = db.query(Table).filter(Table.id == table_id).first()
    if not table:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Table with ID {table_id} not found"
        )
    
    # Get active assignments for this table
    active_assignments = db.query(GroupAssignment).filter(
        GroupAssignment.table_id == table_id,
        GroupAssignment.released_at == None
    ).all()
    
    if not active_assignments:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Table {table_id} is not currently assigned to any customer group"
        )
    
    # Update table status
    table.status = TableStatus.AVAILABLE
    table.updated_at = datetime.utcnow()
    db.add(table)
    
    # Release all assignments
    customer_ids = []
    for assignment in active_assignments:
        assignment.released_at = datetime.utcnow()
        db.add(assignment)
        customer_ids.append(assignment.customer_id)
    
    db.commit()
    
    # Log this action
    customer_str = ", ".join([str(cid) for cid in customer_ids])
    log_info(db, f"테이블 {table_id} 해제됨, 고객 그룹: {customer_str}", background_tasks)
    
    return {
        "status": "success",
        "message": f"Table {table_id} released"
    }

@router.put("/{table_id}/status", response_model=dict)
def update_table_status(
    table_id: int,
    status_data: dict,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    if "status" not in status_data:
        raise HTTPException(status_code=400, detail="Status is required")
        
    new_status = status_data["status"].upper()
    if new_status not in ["AVAILABLE", "OCCUPIED"]:
        raise HTTPException(status_code=400, detail=f"Invalid status: {new_status}")
    
    # Find table
    table = db.query(Table).filter(Table.id == table_id).first()
    if not table:
        raise HTTPException(status_code=404, detail=f"Table with ID {table_id} not found")
    
    # Update status
    old_status = table.status
    table.status = TableStatus(new_status)
    table.updated_at = datetime.utcnow()
    
    db.add(table)
    db.commit()
    
    # Handle inconsistencies with assignments based on new status
    if new_status == "AVAILABLE":
        # If setting to available, release any active assignments
        active_assignments = db.query(GroupAssignment).filter(
            GroupAssignment.table_id == table_id,
            GroupAssignment.released_at == None
        ).all()
        
        if active_assignments:
            for assignment in active_assignments:
                assignment.released_at = datetime.utcnow()
                db.add(assignment)
            db.commit()
            log_warning(db, f"테이블 {table_id} 상태를 AVAILABLE로 변경하여 {len(active_assignments)}개의 배정이 자동으로 해제됨", background_tasks)
    
    # Log status change
    log_info(db, f"테이블 {table_id} 상태 변경: {old_status} → {new_status}", background_tasks)
    
    return {
        "status": "success",
        "message": f"Table status updated to {new_status}"
    }

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
