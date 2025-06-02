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


# 로거 설정 및 저장
import logging
import os

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# —————————————
# 로그 파일 핸들러 설정
# —————————————
log_dir = os.path.join(os.getcwd(), "logs")
os.makedirs(log_dir, exist_ok=True)
file_handler = logging.FileHandler(os.path.join(log_dir, "order.log"))
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(logging.Formatter(
    "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
))
logger.addHandler(file_handler)
# —————————————

# --- Pydantic Schemas ---
class OrderItemRequest(BaseModel):
    menu_item_id: int
    quantity: int
    status: Optional[OrderStatus] = OrderStatus.PLACED

class OrderCreateRequest(BaseModel):
    customer_id: int
    robot_id: Optional[int] = None
    table_id: Optional[int] = None
    items: List[OrderItemRequest]

class OrderItemResponse(BaseModel):
    menu_item_id: int
    quantity: int
    status: Optional[OrderStatus] = OrderStatus.PLACED

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
    # 1) 대기 중인 주문중에 가장 먼저 생성된 주문 중에 COMPLETED,SERVED,CANCELLED가 아닌 주문을 조회합니다.
    order = db.query(Order).order_by(Order.id).filter(Order.status != OrderStatus.CANCELLED).filter(
        Order.status != OrderStatus.SERVED).filter(Order.status != OrderStatus.COMPLETED).first()
    if not order:
        raise HTTPException(status.HTTP_404_NOT_FOUND,
                            detail="대기중인 주문이 없습니다.")
    # 2) 주문 항목을 조회합니다.
    items = db.query(OrderItem).filter(OrderItem.order_id == order.id).filter(OrderItem.status == OrderStatus.PLACED).all()
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
            item_name=menu_item.name,
        ) for item in items for menu_item in menu_items if item.menu_item_id == menu_item.id
    ]

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
            quantity=quantity,
            status=OrderStatus.PLACED
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
    valid_statuses = OrderStatus.__members__.keys()
    
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

    # 취소일시 주문 항목도 취소
    if new_status == "CANCELLED":
        order_items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
        for item in order_items:
            item.status = OrderStatus.CANCELLED
            db.add(item)
            db.commit()
        # Log order item cancellation
        log_info(db,
            f"주문 항목 {order_id} 취소됨",
            background_tasks
        )

    # 주문이 서빙완료일시 전체 주문 항목도 서빙완료로 변경
    if new_status == "SERVED":
        order_items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
        for item in order_items:
            item.status = OrderStatus.SERVED
            db.add(item)
            db.commit()
        # Log order item serving
        log_info(db,
            f"주문 항목 {order_id} 서빙완료됨",
            background_tasks
        )
    
    if new_status == "COMPLETED":
        order_items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
        for item in order_items:
            item.status = OrderStatus.COMPLETED
            db.add(item)
            db.commit()
        # Log order item completion
        log_info(db,
            f"주문 항목 {order_id} 완료됨",
            background_tasks
        )

    # 주문이 준비중일시 전체 주문 항목도 준비중으로 변경 및 핑키에게 서빙 요청 명령
    if new_status == "PREPARING":
        order_items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
        for item in order_items:
            item.status = OrderStatus.PREPARING
            db.add(item)
            db.commit()
        # Log order item preparing
        log_info(db,
            f"주문 항목 {order_id} 준비중됨",
            background_tasks
        )
        command = {
            "robot_id": None,             # 필요에 맞게 설정
            "command": "SERVING",
            "parameters": {
                "order_id": order_id,
                "table_id": order.table_id,
            },
            "status": "PENDING"
        }
        from app.routes.robot import send_command
        send_command(command, db)
    # Log order status change
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

#OrderItem의 상태 업데이트
@router.put("/{order_id}/items/{item_id}/status", response_model=dict)
def update_order_item_status(
    order_id: int,
    item_id: int,
    status_data: OrderStatusUpdateRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Validate input
    if status_data is None:
        raise HTTPException(status_code=400, detail="Status is required")
        
    new_status = status_data.status.upper()
    valid_statuses = OrderStatus.__members__.keys()
    
    if new_status not in valid_statuses:
        raise HTTPException(status_code=400, detail=f"Invalid status: {new_status}")
    
    # Find order item
    order_item = db.query(OrderItem).filter(
        OrderItem.order_id == order_id,
        OrderItem.menu_item_id == item_id
    ).first()
    
    if not order_item:
        raise HTTPException(status_code=404, detail=f"Order item with ID {item_id} not found in order {order_id}")
    
    # Update status
    old_status = order_item.status
    order_item.status = OrderStatus(new_status)
    
    db.add(order_item)
    db.commit()

    # 준비중으로 변경시, 주문 상태도 변경
    if new_status == "PREPARING":
        order = db.query(Order).filter(Order.id == order_id).first()
        if order:
            order.status = OrderStatus.PREPARING
            db.add(order)
            db.commit()
            # Log order status change
            log_info(db,
                f"주문 {order_id} 상태 변경: {old_status} → {new_status}",
                background_tasks
            )
    
    # 모든 주문 항목이 완료 상태일시 주문 상태 변경
    order_items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
    all_prepared = all(item.status == OrderStatus.SERVED for item in order_items)
    if all_prepared:
        order = db.query(Order).filter(Order.id == order_id).first()
        if order:
            order.status = OrderStatus.SERVED
            db.add(order)
            db.commit()
            # Log order status change
            log_info(db,
                f"주문 {order_id} 상태 변경: {old_status} → {new_status}",
                background_tasks
            )

    #모든 주문 항목이 취소 상태일시 주문 상태 변경
    all_cancelled = all(item.status == OrderStatus.CANCELLED for item in order_items)
    if all_cancelled:
        order = db.query(Order).filter(Order.id == order_id).first()
        if order:
            order.status = OrderStatus.CANCELLED
            db.add(order)
            db.commit()
            # Log order status change
            log_info(db,
                f"주문 {order_id} 상태 변경: {old_status} → {new_status}",
                background_tasks
            )

    
    # Log status change
    log_info(db, 
        f"주문 항목 {item_id} 상태 변경: {old_status} → {new_status}",
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
        "message": f"Order item status updated to {new_status}"
    }

# 주문 전체 취소
@router.put("/cancel_order/{customer_id}", response_model=dict)
def cancel_order(
    customer_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # Find order
    order = db.query(Order).filter(Order.customer_id == customer_id).all()
    
    if not order:
        raise HTTPException(status_code=404, detail=f"Order with customer ID {customer_id} not found")

    # 주문 항목 취소
    for o in order:
        order_items = db.query(OrderItem).filter(OrderItem.order_id == o.id).all()
        for item in order_items:
            item.status = OrderStatus.CANCELLED
            db.add(item)
            db.commit()

        # 주문 취소
        o.status = OrderStatus.CANCELLED
        db.add(o)
        db.commit()
        # Log order cancellation
        log_info(db, 
            f"주문 {o.id} 취소됨",
            background_tasks
        )
    # Log cancellation
    log_info(db, 
        f"주문 {customer_id} 취소됨",
        background_tasks
    )
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    from run import broadcast_entity_update
    background_tasks.add_task(
        broadcast_entity_update,
        "order",
        None
    )
    return {
        "status": "success",
        "message": f"Order with customer ID {customer_id} cancelled"
    }
