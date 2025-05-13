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
from app.models.inventory import MenuItem
from app.routes.auth import get_current_user
from app.routes.events import log_info, log_warning, log_error

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


class TodoOrderResponse(BaseModel):
    order_id: int
    item_name: str

# --- Endpoints ---

# 쿡봇 전달용 API
@router.get("/todo_order", response_model=TodoOrderResponse)
def get_todo_order(
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)):
    # 1) 대기 중인 주문중에 가장 먼저 생성된 주문을 조회합니다.
    order = db.query(Order).order_by(Order.id).filter(Order.status == OrderStatus.PLACED).first()
    if not order:
        raise HTTPException(status.HTTP_404_NOT_FOUND,
                            detail="대기 중인 주문이 없습니다.")
    # 2) 주문 항목을 조회합니다.
    items = db.query(OrderItem).filter(OrderItem.order_id == order.id).all()
    if not items:
        raise HTTPException(status.HTTP_404_NOT_FOUND,
                            detail="주문 항목이 없습니다, 대기중인 주문 : %s" % order.id)
    # 3) 메뉴 항목을 조회합니다.
    menu_items = db.query(MenuItem).filter(MenuItem.id.in_([item.menu_item_id for item in items])).all()
    if not menu_items:
        raise HTTPException(status.HTTP_404_NOT_FOUND,
                            detail="메뉴 항목이 없습니다.")
    # 4) 주문 항목과 메뉴 항목을 매핑합니다.
    item_res = [
        TodoOrderResponse(
            order_id=order.id,
            item_name=menu_item.name
        ) for item in items for menu_item in menu_items if item.menu_item_id == menu_item.id
    ]
    # 5) 주문 상태를 업데이트합니다.
    order.status = OrderStatus.PREPARING
    db.add(order)
    db.commit()
    db.refresh(order)
    # 6) 시스템 로그를 생성합니다.
    log = SystemLog(
        level=LogLevel.INFO,
        message=f"주문이 준비 중으로 변경되었습니다. 주문 ID: {order.id}, 테이블: {order.table_id or '없음'}",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    db.refresh(log)
    # 7) 웹소켓 브로드캐스트 예약
    from run import broadcast_entity_update
    background_tasks.add_task(
        broadcast_entity_update,
        "order",
        None,
    )
    # 8) 시스템 로그 브로드캐스트 예약
    background_tasks.add_task(
        broadcast_entity_update,
        "systemlog",
        None,
    )
    # 9) 응답을 반환합니다.
    return TodoOrderResponse(
        order_id=order.id,
        item_name=item_res[0].item_name
    )

@router.get("/todo_order_test", response_model=TodoOrderResponse)
def get_todo_order_test(
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)):

    # 예시 데이터로
    return TodoOrderResponse(
        order_id=1,
        item_name="salad"
    )


# 주문 목록 조회
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

@router.post("", response_model=dict)
def create_order(
    order_data: dict,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Validate data
    if "customer_id" not in order_data:
        raise HTTPException(status_code=400, detail="Customer ID is required")
    
    if "items" not in order_data or not order_data["items"]:
        raise HTTPException(status_code=400, detail="Order items are required")
    
    # Check if customer exists
    customer_id = order_data["customer_id"]
    customer = db.query(Customer).filter(Customer.id == customer_id).first()
    if not customer:
        raise HTTPException(status_code=404, detail=f"Customer with ID {customer_id} not found")
    
    # Get table ID if the customer is assigned to a table
    table_id = None
    from app.models import GroupAssignment
    assignment = db.query(GroupAssignment).filter(
        GroupAssignment.customer_id == customer_id,
        GroupAssignment.released_at == None  # Only currently active assignments
    ).first()
    
    if assignment:
        table_id = assignment.table_id
    
    # Create order
    order = Order(
        customer_id=customer_id,
        table_id=table_id,
        status=OrderStatus.PLACED,
        timestamp=datetime.utcnow()
    )
    
    db.add(order)
    db.commit()
    db.refresh(order)
    
    # Create order items
    total_items = 0
    items_summary = []
    
    for item_data in order_data["items"]:
        if "menu_item_id" not in item_data or "quantity" not in item_data:
            continue
            
        menu_item_id = item_data["menu_item_id"]
        quantity = item_data["quantity"]
        
        # Verify menu item exists
        menu_item = db.query(MenuItem).filter(MenuItem.id == menu_item_id).first()
        if not menu_item:
            continue
            
        order_item = OrderItem(
            order_id=order.id,
            menu_item_id=menu_item_id,
            quantity=quantity
        )
        
        db.add(order_item)
        total_items += quantity
        items_summary.append(f"{menu_item.name} x{quantity}")
    
    db.commit()

    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "order",
        None
    )
    
    # Log order creation
    items_str = ", ".join(items_summary)
    log_info(db, 
        f"주문 생성: ID {order.id}, 고객 {customer_id}, 테이블 {table_id or '없음'}, 항목: {items_str}",
        background_tasks
    )
    
    return {
        "id": order.id,
        "status": "success",
        "message": "Order created successfully"
    }

@router.put("/{order_id}/status", response_model=dict)
def update_order_status(
    order_id: int,
    status_data: dict,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Validate input
    if "status" not in status_data:
        raise HTTPException(status_code=400, detail="Status is required")
        
    new_status = status_data["status"].upper()
    valid_statuses = ["PLACED", "PREPARING", "SERVED", "CANCELLED"]
    
    if new_status not in valid_statuses:
        raise HTTPException(status_code=400, detail=f"Invalid status: {new_status}")
    
    # Find order
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(status_code=404, detail=f"Order with ID {order_id} not found")
    
    # Update status
    old_status = order.status
    order.status = OrderStatus(new_status)
    
    # If status is SERVED, update served_at timestamp
    if new_status == "SERVED":
        order.served_at = datetime.utcnow()
    
    db.add(order)
    db.commit()
    
    # Log status change
    log_info(db, 
        f"주문 {order_id} 상태 변경: {old_status} → {new_status}",
        background_tasks
    )

    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "order",
        None
    )
    
    return {
        "status": "success",
        "message": f"Order status updated to {new_status}"
    }

@router.post("/{order_id}/assign-robot", response_model=dict)
def assign_robot_to_order(
    order_id: int,
    robot_data: dict,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    """로봇에 주문 배정"""
    # Validate input
    if "robot_id" not in robot_data:
        raise HTTPException(status_code=400, detail="Robot ID is required")
        
    robot_id = robot_data["robot_id"]
    
    # Find order
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(status_code=404, detail=f"Order with ID {order_id} not found")
    
    # Check if order status is valid for robot assignment
    if order.status != OrderStatus.PREPARING:
        raise HTTPException(
            status_code=400, 
            detail=f"Order must be in PREPARING status to assign a robot (current: {order.status})"
        )
    
    # Check if robot exists
    from app.models import Robot
    robot = db.query(Robot).filter(Robot.robot_id == robot_id).first()
    if not robot:
        raise HTTPException(status_code=404, detail=f"Robot with ID {robot_id} not found")
    
    # Check if robot is available
    from app.models import Albabot
    from app.models.enums import RobotStatus
    
    albabot = db.query(Albabot).filter(Albabot.robot_id == robot_id).order_by(Albabot.id.desc()).first()
    if not albabot or albabot.status != RobotStatus.IDLE:
        raise HTTPException(
            status_code=400, 
            detail=f"Robot {robot_id} is not available (status: {albabot.status if albabot else 'unknown'})"
        )
    
    # Assign robot to order
    order.robot_id = robot_id
    db.add(order)
    db.commit()
    
    # Create new robot status
    new_albabot = Albabot(
        robot_id=robot_id,
        status=RobotStatus.SERVING,
        battery_level=albabot.battery_level,
        timestamp=datetime.utcnow()
    )
    db.add(new_albabot)
    db.commit()

    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "order",
        None
    )
    
    # Log assignment
    log_info(db, 
        f"로봇 {robot_id} 주문 {order_id}에 배정되었습니다.",
        background_tasks
    )
    
    return {
        "status": "success",
        "message": f"Robot {robot_id} assigned to order {order_id}"
    }
