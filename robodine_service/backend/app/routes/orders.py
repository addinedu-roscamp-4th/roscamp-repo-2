# app/routes/orders.py
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Order, OrderItem, Customer, Robot, KioskTerminal
from app.models.enums import OrderStatus

router = APIRouter()

# --- Pydantic Schemas ---
class OrderItemRequest(BaseModel):
    menu_item_id: int
    quantity: int

class OrderCreateRequest(BaseModel):
    customer_id: int
    robot_id: Optional[int] = None
    kiosk_id: Optional[int] = None
    items: List[OrderItemRequest]

class OrderItemResponse(BaseModel):
    menu_item_id: int
    quantity: int

class OrderResponse(BaseModel):
    id: int
    customer_id: int
    robot_id: Optional[int]
    kiosk_id: Optional[int]
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
    kiosk_id: Optional[int]
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
            kiosk_id=o.kiosk_id,
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
        kiosk_id=o.kiosk_id,
        items=item_res,
        status=o.status,
        timestamp=o.timestamp,
        served_at=o.served_at
    )

@router.post("", response_model=OrderResponse, status_code=status.HTTP_201_CREATED)
def create_order(data: OrderCreateRequest, db: Session = Depends(get_db)):
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
    # 3) (선택) KioskTerminal 검증
    if data.kiosk_id is not None:
        ks = db.query(KioskTerminal).get(data.kiosk_id)
        if not ks:
            raise HTTPException(status.HTTP_404_NOT_FOUND,
                                detail=f"KioskTerminal {data.kiosk_id} not found")

    # 4) Order 생성
    new_o = Order(
        customer_id=data.customer_id,
        robot_id=data.robot_id,
        kiosk_id=data.kiosk_id,
        status=OrderStatus.PLACED
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

    # 6) 방금 추가된 OrderItem 조회
    items = db.query(OrderItem).filter(OrderItem.order_id == new_o.id).all()
    item_res = [
        OrderItemResponse(menu_item_id=i.menu_item_id, quantity=i.quantity)
        for i in items
    ]

    return OrderResponse(
        id=new_o.id,
        customer_id=new_o.customer_id,
        robot_id=new_o.robot_id,
        kiosk_id=new_o.kiosk_id,
        items=item_res,
        status=new_o.status,
        timestamp=new_o.timestamp,
        served_at=new_o.served_at
    )

@router.put("/{order_id}/status", response_model=OrderResponse)
def update_order_status(order_id: int,
                        req: OrderStatusUpdateRequest,
                        db: Session = Depends(get_db)):
    o = db.query(Order).filter(Order.id == order_id).first()
    if not o:
        raise HTTPException(status.HTTP_404_NOT_FOUND,
                            detail=f"Order {order_id} not found")

    o.status = req.status
    if req.status == OrderStatus.SERVED:
        o.served_at = datetime.utcnow()
    db.add(o)
    db.commit()
    db.refresh(o)

    items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
    item_res = [
        OrderItemResponse(menu_item_id=i.menu_item_id, quantity=i.quantity)
        for i in items
    ]
    return OrderResponse(
        id=o.id,
        customer_id=o.customer_id,
        robot_id=o.robot_id,
        kiosk_id=o.kiosk_id,
        items=item_res,
        status=o.status,
        timestamp=o.timestamp,
        served_at=o.served_at
    )
