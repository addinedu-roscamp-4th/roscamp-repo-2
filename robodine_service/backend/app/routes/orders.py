# app/routes/orders.py
from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Order, OrderItem, Customer, Robot, KioskTerminal
from app.models.enums import OrderStatus
from app.models.event import SystemLog, LogLevel

router = APIRouter()

# --- Pydantic Schemas ---
class OrderItemRequest(BaseModel):
    menu_item_id: int
    quantity: int

class OrderCreateRequest(BaseModel):
    customer_id: int
    robot_id: Optional[int] = None
    table_id: Optional[int] = None
    items: List[OrderItemRequest]

class OrderItemResponse(BaseModel):
    menu_item_id: int
    quantity: int

class OrderResponse(BaseModel):
    id: int
    customer_id: int
    robot_id: Optional[int]
    table_id: Optional[int]
    items: List[OrderItemResponse]
    status: OrderStatus
    timestamp: datetime
    served_at: Optional[datetime]

    class Config:
        orm_mode = True

class OrderListResponse(BaseModel):
    id: int
    customer_id: int
    robot_id: Optional[int]
    table_id: Optional[int]
    status: OrderStatus
    timestamp: datetime

    class Config:
        orm_mode = True

class OrderStatusUpdateRequest(BaseModel):
    status: OrderStatus

# --- Endpoints ---
@router.get("", response_model=List[OrderListResponse])
def get_orders(db: Session = Depends(get_db)):
    orders = db.query(Order).all()
    return [
        OrderListResponse(
            id=o.id,
            customer_id=o.customer_id,
            robot_id=o.robot_id,
            table_id=o.table_id,
            status=o.status,
            timestamp=o.timestamp
        ) for o in orders
    ]

@router.get("/{order_id}", response_model=OrderResponse)
def get_order(order_id: int, db: Session = Depends(get_db)):
    o = db.query(Order).filter(Order.id == order_id).first()
    if not o:
        raise HTTPException(status.HTTP_404_NOT_FOUND,
                            detail=f"Order {order_id} not found")

    items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
    item_res = [
        OrderItemResponse(menu_item_id=i.menu_item_id, quantity=i.quantity)
        for i in items
    ]
    return OrderResponse(
        id=o.id,
        customer_id=o.customer_id,
        robot_id=o.robot_id,
        table_id=o.table_id,
        items=item_res,
        status=o.status,
        timestamp=o.timestamp,
        served_at=o.served_at
    )

@router.post("", response_model=OrderResponse, status_code=status.HTTP_201_CREATED)
def create_order(
    data: OrderCreateRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)):
    # 1) Customer 검증
    cust = db.query(Customer).get(data.customer_id)

    if not cust:
        raise HTTPException(status.HTTP_404_NOT_FOUND,
                            detail=f"Customer {data.customer_id} not found")
    # 2) (선택) Robot 검증
    if data.robot_id is not None:
        rob = db.query(Robot).get(data.robot_id)
        if not rob:
            raise HTTPException(status.HTTP_404_NOT_FOUND,
                                detail=f"Robot {data.robot_id} not found")

    # 4) Order 생성
    new_o = Order(
        customer_id=data.customer_id,
        robot_id=data.robot_id,
        table_id=data.table_id,
        status=OrderStatus.PREPARING,
        # timestamp: default_factory에 의해 자동 설정
    )
    db.add(new_o)
    db.commit()
    db.refresh(new_o)

    # 5) OrderItem 생성
    for it in data.items:
        oi = OrderItem(
            order_id=new_o.id,
            menu_item_id=it.menu_item_id,
            quantity=it.quantity
        )
        db.add(oi)
    db.commit()

    # 6) 시스템 로그 생성
    log = SystemLog(
        level=LogLevel.INFO,
        message=f"새로운 주문이 생성되었습니다. 주문 ID: {new_o.id}, 테이블: {new_o.table_id or '없음'}, 품목 수: {len(data.items)}개",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    db.refresh(log)

    # **1) 주문 항목을 직접 다시 조회합니다**
    items = db.query(OrderItem).filter(OrderItem.order_id == new_o.id).all()

    # 2) 웹소켓 브로드캐스트 예약
    from run import broadcast_entity_update
    background_tasks.add_task(
        broadcast_entity_update,
        "order",
        None,
    )
    # 시스템 로그 브로드캐스트 예약
    background_tasks.add_task(
        broadcast_entity_update,
        "systemlog",
        None,
    )

    # 3) Pydantic 응답으로 리턴
    return OrderResponse(
        id=new_o.id,
        customer_id=new_o.customer_id,
        robot_id=new_o.robot_id,
        table_id=new_o.table_id,
        items=[
            OrderItemResponse(menu_item_id=i.menu_item_id, quantity=i.quantity)
            for i in items
        ],
        status=new_o.status,
        timestamp=new_o.timestamp,
        served_at=new_o.served_at
    )

@router.put("/{order_id}/status", response_model=OrderResponse)
def update_order_status(order_id: int,
                        req: OrderStatusUpdateRequest,
                        background_tasks: BackgroundTasks,
                        db: Session = Depends(get_db)):
    o = db.query(Order).filter(Order.id == order_id).first()
    if not o:
        raise HTTPException(status.HTTP_404_NOT_FOUND,
                            detail=f"Order {order_id} not found")

    old_status = o.status
    o.status = req.status
    if req.status == OrderStatus.SERVED:
        o.served_at = datetime.utcnow()
    db.add(o)
    db.commit()
    db.refresh(o)

    # 시스템 로그 생성
    log_level = LogLevel.INFO
    if req.status == OrderStatus.SERVED:
        log_level = LogLevel.INFO
        log_message = f"주문이 서빙 완료되었습니다. 주문 ID: {order_id}, 테이블: {o.table_id or '없음'}"
    elif req.status == OrderStatus.PREPARING:
        log_level = LogLevel.INFO
        log_message = f"주문이 준비 중입니다. 주문 ID: {order_id}, 테이블: {o.table_id or '없음'}"
    elif req.status == OrderStatus.PLACED:
        log_level = LogLevel.INFO
        log_message = f"주문이 접수되었습니다. 주문 ID: {order_id}, 테이블: {o.table_id or '없음'}"
    elif req.status == OrderStatus.CANCELLED:
        log_level = LogLevel.WARNING
        log_message = f"주문이 취소되었습니다. 주문 ID: {order_id}, 테이블: {o.table_id or '없음'}"
    else:
        log_message = f"주문 상태가 변경되었습니다. 주문 ID: {order_id}, {old_status} → {req.status}"

    log = SystemLog(
        level=log_level,
        message=log_message,
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    db.refresh(log)

    items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
    item_res = [
        OrderItemResponse(menu_item_id=i.menu_item_id, quantity=i.quantity)
        for i in items
    ]

    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "order",
        None,
    )
    # 시스템 로그 브로드캐스트 예약
    background_tasks.add_task(
        broadcast_entity_update,
        "systemlog",
        None,
    )

    return OrderResponse(
        id=o.id,
        customer_id=o.customer_id,
        robot_id=o.robot_id,
        table_id=o.table_id,
        items=item_res,
        status=o.status,
        timestamp=o.timestamp,
        served_at=o.served_at
    )
