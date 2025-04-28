from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional, Dict
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import Order, OrderItem, Customer
from app.models.enums import OrderStatus

router = APIRouter()

# --- Order Models ---
class OrderItemRequest(BaseModel):
    menu_item_id: int
    quantity: int

class OrderCreateRequest(BaseModel):
    customer_id: int
    items: List[OrderItemRequest]

class OrderItemResponse(BaseModel):
    menu_item_id: int
    quantity: int

class OrderResponse(BaseModel):
    id: int
    customer_id: int
    items: List[OrderItemResponse]
    status: OrderStatus
    ordered_at: datetime
    served_at: Optional[datetime] = None

class OrderListResponse(BaseModel):
    id: int
    customer_id: int
    status: OrderStatus
    ordered_at: datetime

class OrderStatusUpdateRequest(BaseModel):
    status: OrderStatus

# --- Router Endpoints ---
@router.get("", response_model=List[OrderListResponse])
def get_orders(db: Session = Depends(get_db)):
    orders = db.query(Order).all()
    return [
        OrderListResponse(
            id=order.id,
            customer_id=order.customer_id,
            status=order.status,
            ordered_at=order.ordered_at
        ) for order in orders
    ]

@router.get("/{order_id}", response_model=OrderResponse)
def get_order(order_id: int, db: Session = Depends(get_db)):
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Order with ID {order_id} not found"
        )
    
    # Get order items
    order_items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
    
    return OrderResponse(
        id=order.id,
        customer_id=order.customer_id,
        items=[
            OrderItemResponse(
                menu_item_id=item.menu_item_id,
                quantity=item.quantity
            ) for item in order_items
        ],
        status=order.status,
        ordered_at=order.ordered_at,
        served_at=order.served_at
    )

@router.post("", response_model=dict)
def create_order(order_data: OrderCreateRequest, db: Session = Depends(get_db)):
    # Check if customer exists
    customer = db.query(Customer).filter(Customer.id == order_data.customer_id).first()
    if not customer:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Customer with ID {order_data.customer_id} not found"
        )
    
    # Create new order
    new_order = Order(
        customer_id=order_data.customer_id,
        status=OrderStatus.PLACED,
        ordered_at=datetime.utcnow()
    )
    
    db.add(new_order)
    db.commit()
    db.refresh(new_order)
    
    # Create order items
    for item in order_data.items:
        order_item = OrderItem(
            order_id=new_order.id,
            menu_item_id=item.menu_item_id,
            quantity=item.quantity
        )
        db.add(order_item)
    
    db.commit()
    
    return {"id": new_order.id, "status": "success"}

@router.put("/{order_id}/status")
def update_order_status(order_id: int, status_data: OrderStatusUpdateRequest, db: Session = Depends(get_db)):
    # Find order
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Order with ID {order_id} not found"
        )
    
    # Update status
    order.status = status_data.status
    
    # If status is SERVED, update served_at timestamp
    if status_data.status == OrderStatus.SERVED:
        order.served_at = datetime.utcnow()
    
    db.add(order)
    db.commit()
    
    return {"status": "success", "message": "주문 상태가 변경되었습니다."}
