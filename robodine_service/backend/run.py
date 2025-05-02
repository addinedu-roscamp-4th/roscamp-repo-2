# robodine_service/backend/run.py

import logging
import threading
import socketserver
import json
import asyncio
from datetime import datetime
from contextlib import asynccontextmanager
import anyio

from fastapi import FastAPI, BackgroundTasks, Depends
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy import func
from sqlmodel import SQLModel, Session, select, case

from app.routes import (
    poses, websockets, streaming, inventories,
    robot, albabot, cookbot, auth, users, settings, customers,
    tables, kiosks, orders, menu, events, emergencies, 
    video_streams
)
from app.routes.websockets import router as websocket_router
from app.routes.websockets import broadcast_robots_update, broadcast_tables_update, broadcast_events_update, broadcast_orders_update
from app.routes.websockets import manager
from app.routes.streaming import router as streaming_router
from app.core.db_config import engine
from app.core.database import get_session
from app.models.robot import Robot
from app.models.table import Table
from app.models.event import Event, SystemLog
from app.models.order import Order, OrderItem, KioskTerminal
from app.models.pose6d import Pose6D
from app.models.albabot import Albabot
from app.models.cookbot import Cookbot
from app.models.inventory import Inventory, MenuItem, MenuIngredient
from app.core.utils import dispatch_payload

from app.services.streaming_service import get_stream_urls, add_stream_url

main_loop = None

# 로깅 설정
logger = logging.getLogger("robodine.run")
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# 폴링 주기 설정 (비상 시 폴백용, 초 단위)
POLLING_INTERVAL = 10

# 마지막 브로드캐스트 시간 추적 (중복 방지용)
last_broadcast = {
    "robots": datetime.min,
    "tables": datetime.min,
    "events": datetime.min,
    "orders": datetime.min,
    "status": datetime.min,
    "systemlogs": datetime.min
}

# 모델 데이터 변환 함수들
def serialize_robot(robot):
    """Robot 모델을 JSON 직렬화 가능한 형태로 변환"""
    robot_type = str(robot.type) if robot.type else None
    # 'EntityType.'을 제거하여 타입만 출력
    if robot_type:
        robot_type = robot_type.replace('EntityType.', '')
    return {
        "Robot.id": robot.id,
        "Robot.robot_id": robot.robot_id,
        "Robot.type": robot_type,
        "Robot.mac_address": robot.mac_address,
        "Robot.ip_address": robot.ip_address,
        "Robot.timestamp": robot.timestamp.isoformat() if robot.timestamp else None
    }

def serialize_table(table):
    """Table 모델을 JSON 직렬화 가능한 형태로 변환"""
    table_status = str(table.status) if table.status else None
    if table_status:
        table_status = table_status.replace('TableStatus.', '')
    return {
        "Table.id": table.id,
        "Table.table_number": table.table_number,
        "Table.max_customer": table.max_customer,
        "Table.status": table_status,
        "Table.updated_at": table.updated_at.isoformat() if table.updated_at else None
    }

def serialize_event(event):
    """Event 모델을 JSON 직렬화 가능한 형태로 변환"""
    event_type = str(event.type) if event.type else None
    related_entity_type = str(event.related_entity_type) if event.related_entity_type else None
    if event_type:
        event_type = event_type.replace('EventType.', '')
    if related_entity_type:
        related_entity_type = related_entity_type.replace('EntityType.', '')
    return {
        "Event.id": event.id,
        "Event.type": event_type,
        "Event.related_entity_type": related_entity_type,
        "Event.related_entity_id": event.related_entity_id,
        "Event.description": event.description,
        "Event.timestamp": event.timestamp.isoformat() if event.timestamp else None
    }

def serialize_order(order):
    """Order 모델을 JSON 직렬화 가능한 형태로 변환"""
    order_status = str(order.status) if order.status else None
    if order_status:
        order_status = order_status.replace('OrderStatus.', '')
    return {
        "Order.id": order.id,
        "Order.customer_id": order.customer_id,
        "Order.robot_id": order.robot_id,
        "Order.table_id": order.table_id, 
        "Order.status": order_status,
        "Order.timestamp": order.timestamp.isoformat() if order.timestamp else None,
        "Order.served_at": order.served_at.isoformat() if order.served_at else None
    }

def serialize_pose6d(pose):
    """Pose6D 모델을 JSON 직렬화 가능한 형태로 변환"""
    pose_type = str(pose.entity_type) if pose.entity_type else None
    if pose_type:
        pose_type = pose_type.replace('EntityType.', '')
    return {
        "Pose6D.id": pose.id,
        "Pose6D.entity_id": pose.entity_id,
        "Pose6D.entity_type": pose_type,
        "Pose6D.timestamp": pose.timestamp.isoformat() if pose.timestamp else None,
        "Pose6D.x": pose.x,
        "Pose6D.y": pose.y,
        "Pose6D.z": pose.z,
        "Pose6D.roll": pose.roll,
        "Pose6D.pitch": pose.pitch,
        "Pose6D.yaw": pose.yaw
    }

def serialize_albabot(robot):
    """Albabot 모델을 JSON 직렬화 가능한 형태로 변환"""
    robot_status = str(robot.status) if robot.status else None
    if robot_status:
        robot_status = robot_status.replace('RobotStatus.', '')
    return {
        "Albabot.id": robot.id,
        "Albabot.robot_id": robot.robot_id,
        "Albabot.status": robot_status,
        "Albabot.battery_level": robot.battery_level,
        "Albabot.timestamp": robot.timestamp.isoformat() if robot.timestamp else None
    }

def serialize_cookbot(robot):
    """Cookbot 모델을 JSON 직렬화 가능한 형태로 변환"""
    robot_status = str(robot.status) if robot.status else None
    if robot_status:
        robot_status = robot_status.replace('RobotStatus.', '')
    return {
        "Cookbot.id": robot.id,
        "Cookbot.robot_id": robot.robot_id,
        "Cookbot.status": robot_status,
        "Cookbot.timestamp": robot.timestamp.isoformat() if robot.timestamp else None
    }

def serialize_kioskterminal(kioskterminal):
    """KioskTerminal 모델을 JSON 직렬화 가능한 형태로 변환"""
    return {
        "KioskTerminal.id": kioskterminal.id,
        "KioskTerminal.table_number": kioskterminal.table_number,
        "KioskTerminal.ip_address": kioskterminal.ip_address
    }

def serialize_orderitem(orderitem):
    """OrderItem 모델을 JSON 직렬화 가능한 형태로 변환"""
    return {
        "OrderItem.order_id": orderitem.order_id,
        "OrderItem.menu_item_id": orderitem.menu_item_id,
        "OrderItem.quantity": orderitem.quantity
    }

def serialize_menuitem(menuitem):
    """MenuItem 모델을 JSON 직렬화 가능한 형태로 변환"""
    return {
        "MenuItem.id": menuitem.id,
        "MenuItem.name": menuitem.name,
        "MenuItem.price": menuitem.price,
        "MenuItem.prepare_time": menuitem.prepare_time
    }

def serialize_systemlog(log):
    """SystemLog 모델을 JSON 직렬화 가능한 형태로 변환"""
    return {
        "SystemLog.id": log.id,
        "SystemLog.level": log.level,
        "SystemLog.message": log.message,
        "SystemLog.timestamp": log.timestamp.isoformat() if log.timestamp else None
    }

# 서버 측에서 사용할 데이터 전송 함수들
async def broadcast_robots_update(robots_data):
    """로봇 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(robots_data, "robots")

async def broadcast_status_update(status_data):
    """상태 데이터 업데이트를 브로드캐스팅"""
    message = {
        "type": "update",
        "topic": "status",
        "data": status_data
    }
    await manager.broadcast(message, "status")

async def broadcast_tables_update(tables_data):
    """테이블 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(tables_data, "tables")

async def broadcast_events_update(events_data):
    """이벤트 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(events_data, "events")

async def broadcast_orders_update(orders_data):
    """주문 데이터 업데이트를 브로드캐스팅"""
    message = {
        "type": "update",
        "topic": "orders",
        "data": orders_data
    }
    await manager.broadcast(message, "orders")

async def broadcast_systemlogs_update(logs_data):
    """시스템 로그 데이터 업데이트를 브로드캐스팅"""
    message = {
        "type": "update",
        "topic": "systemlogs",
        "data": logs_data
    }
    await manager.broadcast(message, "systemlogs")

# 비동기 브로드캐스팅 함수
async def broadcast_entity_update(entity_type, entity_id):
    """엔티티 변경 시 해당 데이터를 웹소켓으로 브로드캐스팅"""
    try:
        # 1) DB 조회를 별도 스레드에서 실행
        def _sync_db_work():
            with Session(engine) as session:
                out = {"entity": entity_type}
                # robot
                if entity_type == "robot":
                    # 단일 또는 전체 조회
                    if entity_id:
                        robot = session.get(Robot, entity_id)
                        out['data'] = [serialize_robot(robot)] if robot else []
                    else:
                        robots = session.exec(
                            select(Robot)
                            .order_by(Robot.robot_id)
                            ).all()
                        out['data'] = [serialize_robot(robot) for robot in robots]
                    # 상세 상태 조회 (albabot, cookbot, pose6d)
                    albabots = session.exec(
                        select(Albabot)
                        .order_by(Albabot.robot_id, Albabot.id.desc())
                        .distinct(Albabot.robot_id)
                    ).all()
                    cookbots = session.exec(
                        select(Cookbot)
                        .order_by(Cookbot.robot_id, Cookbot.id.desc())
                        .distinct(Cookbot.robot_id)
                    ).all()
                    poses = session.exec(
                        select(Pose6D)
                        .where(Pose6D.entity_type == "WORLD")
                        .order_by(Pose6D.entity_id, Pose6D.id.desc())
                        .distinct(Pose6D.entity_id)
                    ).all()
                    out['albabots'] = albabots
                    out['cookbots'] = cookbots
                    out['poses'] = poses
                # table
                elif entity_type == "table":
                    if entity_id:
                        table = session.get(Table, entity_id)
                        out['data'] = [serialize_table(table)] if table else []
                    else:
                        tables = session.exec(select(Table)).all()
                        out['data'] = [serialize_table(table) for table in tables]
                # event
                elif entity_type == "event":
                    if entity_id:
                        event = session.get(Event, entity_id)
                        out['data'] = [serialize_event(event)] if event else []
                    else:
                        events = session.exec(
                            select(Event)
                            .order_by(Event.timestamp.desc())
                            .limit(20)
                        ).all()
                        out['data'] = [serialize_event(event) for event in events]
                # order
                elif entity_type == "order":
                    if entity_id:
                        order = session.get(Order, entity_id)
                        out['data'] = [serialize_order(order)] if order else []
                    else:
                        orders = session.exec(
                            select(Order)
                            .order_by(
                                case(
                                    (Order.status == "PREPARING", 0),
                                    else_=1
                                ),
                                Order.id.desc())
                            .limit(20)
                        ).all()
                        out['data'] = [serialize_order(order) for order in orders]
                    # # 주문 아이템 및 키오스크 단말기 정보 추가
                    orderitems = session.exec(
                        select(OrderItem)
                        .order_by(OrderItem.order_id.desc())
                        .distinct(OrderItem.order_id)
                    ).all()
                    out['orderitems'] = orderitems
                    kioskterminals = session.exec(
                        select(KioskTerminal)
                        .order_by(KioskTerminal.id.desc())
                    ).all()
                    out['kioskterminals'] = kioskterminals
                    menuitems = session.exec(
                        select(MenuItem)
                        .order_by(MenuItem.id)
                    ).all()
                    out['menuitems'] = menuitems
                # pose6d
                elif entity_type == "pose6d":
                    poses = session.exec(
                        select(Pose6D)
                        .where(Pose6D.entity_type == "WORLD")
                        .order_by(Pose6D.entity_id, Pose6D.id.desc())
                        .distinct(Pose6D.entity_id)
                    ).all()
                    out['poses'] = poses
                # albabot or cookbot triggers full status
                elif entity_type in ("albabot","cookbot"):
                    # full status: robot 기본정보 + 로봇별 최신 Albabot/Cookbot/Pose6D
                    out['robots'] = session.exec(
                        select(Robot)
                        .order_by(Robot.robot_id)
                        ).all()
                    out['albabots'] = session.exec(
                        select(Albabot)
                        .order_by(Albabot.robot_id, Albabot.id.desc())
                        .distinct(Albabot.robot_id)
                    ).all()
                    out['cookbots'] = session.exec(
                        select(Cookbot)
                        .order_by(Cookbot.robot_id, Cookbot.id.desc())
                        .distinct(Cookbot.robot_id)
                    ).all()
                    out['poses'] = session.exec(
                        select(Pose6D)
                        .where(Pose6D.entity_type=="WORLD")
                        .order_by(Pose6D.entity_id, Pose6D.id.desc())
                        .distinct(Pose6D.entity_id)
                    ).all()
                # systemlog
                elif entity_type == "systemlog":
                    if entity_id:
                        log = session.get(SystemLog, entity_id)
                        out['data'] = [serialize_systemlog(log)] if log else []
                    else:
                        logs = session.exec(
                            select(SystemLog)
                            .order_by(SystemLog.timestamp.desc())
                            .limit(30)
                        ).all()
                        out['data'] = [serialize_systemlog(log) for log in logs]
                return out

        result = await anyio.to_thread.run_sync(_sync_db_work)
        now = datetime.now()

        # 2) 스레드에서 반환된 결과로 브로드캐스트
        entity = result['entity']
        if entity == "robot":
            await broadcast_robots_update(result['data'])
            last_broadcast['robots'] = now
            combined = {
                'robots': result['data'],
                'albabots': [serialize_albabot(albabots) for albabots in result['albabots']],
                'cookbots': [serialize_cookbot(cookbots) for cookbots in result['cookbots']],
                'poses': [serialize_pose6d(poses) for poses in result['poses']],
            }
            await broadcast_status_update(combined)
            last_broadcast['status'] = now
        elif entity == "table":
            await broadcast_tables_update(result['data'])
            last_broadcast['tables'] = now
        elif entity == "event":
            await broadcast_events_update(result['data'])
            last_broadcast['events'] = now
        elif entity == "order":
            combined = {
                'orders': [result['data']],
                'kioskterminals': [serialize_kioskterminal(kioskterminals) for kioskterminals in result['kioskterminals']],
                'orderitems' : [serialize_orderitem(orderitems) for orderitems in result['orderitems']],
                'menuitems' : [serialize_menuitem(menuitems) for menuitems in result['menuitems']],
            }
            await broadcast_orders_update(combined)
            last_broadcast['orders'] = now
        elif entity == entity in ("albabot","cookbot"):
            combined = {
                'robots': [serialize_robot(robots) for robots in result.get('robots',[])],
                'albabots': [serialize_albabot(albabots) for albabots in result.get('albabots',[])],
                'cookbots': [serialize_cookbot(cookbots) for cookbots in result.get('cookbots',[])],
                'poses': [serialize_pose6d(poses) for poses in result.get('poses',[])],
            }
            await broadcast_status_update(combined)
            last_broadcast['status'] = now
        elif entity == "systemlog":
            await broadcast_systemlogs_update(result['data'])
            last_broadcast['systemlogs'] = now

    except Exception as entity:
        logger.error(f"Error in broadcasting {entity_type} update: {entity}")

# 백업 폴링 함수 - 변경 감지 실패 대비 (10초 주기)
async def fallback_polling():
    """데이터베이스 폴링 백업 메커니즘 (이벤트 놓침 대비)"""
    logger.info("Starting fallback polling")
    while True:
        try:
            # 각 엔티티 타입에 대해 마지막 브로드캐스트 이후 일정 시간이 지났으면 갱신
            now = datetime.now()
            
            # 로봇 데이터 10초마다 갱신
            if (now - last_broadcast["robots"]).total_seconds() > POLLING_INTERVAL:
                await broadcast_entity_update("robot", None)
                
            # 테이블 데이터 10초마다 갱신
            if (now - last_broadcast["tables"]).total_seconds() > POLLING_INTERVAL:
                await broadcast_entity_update("table", None)
                
            # 이벤트 데이터 10초마다 갱신
            if (now - last_broadcast["events"]).total_seconds() > POLLING_INTERVAL:
                await broadcast_entity_update("event", None)
                
            # 주문 데이터 10초마다 갱신
            if (now - last_broadcast["orders"]).total_seconds() > POLLING_INTERVAL:
                await broadcast_entity_update("order", None)

            # fallback_polling 에서 status도 주기 갱신
            if (now - last_broadcast["status"]).total_seconds() > POLLING_INTERVAL:
                await broadcast_entity_update("robot", None)   # robot→status 통합 로직 사용
            
            # 시스템 로그 데이터 10초마다 갱신
            if (now - last_broadcast["systemlogs"]).total_seconds() > POLLING_INTERVAL:
                await broadcast_entity_update("systemlog", None)
                
        except Exception as e:
            logger.error(f"Error in fallback polling: {str(e)}")
            
        await asyncio.sleep(POLLING_INTERVAL)

@asynccontextmanager
async def lifespan(app: FastAPI):
    global main_loop
    main_loop = asyncio.get_event_loop()
    logger.info("Starting up application...")
    SQLModel.metadata.create_all(bind=engine)
    logger.info("Database tables created successfully")

    # TCP 서버
    tcp_thread = threading.Thread(target=start_tcp_server, args=("0.0.0.0", 8001), daemon=True)
    tcp_thread.start()

    # 백업 폴링 시작 (10초마다 실행)
    fallback_task = asyncio.create_task(fallback_polling())
    logger.info("Started fallback polling for data updates")

    # 초기 RTSP 녹화 시작 (config로 미리 등록된 URL이 있으면)
    for url in get_stream_urls():
        # 이미 서비스 모듈에서 쓰레드를 띄우도록 설계했으니 단순 get만
        pass

    # 마지막 브로드캐스트 시간에 systemlogs 추가
    last_broadcast["systemlogs"] = datetime.min

    yield
    
    # 애플리케이션 종료 시 태스크 취소
    fallback_task.cancel()
    try:
        await fallback_task
    except asyncio.CancelledError:
        logger.info("Fallback polling task cancelled")
    
    logger.info("Shutting down application...")

app = FastAPI(lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# TCP 핸들러에서 웹소켓 브로드캐스팅 추가
class RoboDineTCPHandler(socketserver.BaseRequestHandler):
    def handle(self):
        raw = self.request.recv(1024).strip()
        logger.info(f"[TCP] Received: {raw}")
        try:
            payload = json.loads(raw.decode('utf-8'))
            entity_type = None
            entity_id = None
            
            with get_session() as session:
                # msg_type에 따라 분기 처리 및 관련 엔티티 식별
                result = dispatch_payload(session, payload)
                
                # dispatch_payload 결과에서 영향받은 엔티티 정보 추출
                if isinstance(result, dict) and 'affected_entity' in result:
                    entity_type = result['affected_entity'].get('type')
                    entity_id = result['affected_entity'].get('id')
                    print (f"Entity Type: {entity_type}, Entity ID: {entity_id}")
                
                session.commit()
                
                # 변경 후 즉시 비동기 브로드캐스팅 작업 생성
                if entity_type:
                    # FastAPI 메인 이벤트 루프에서 직접 태스크 생성
                    if main_loop:
                        main_loop.call_soon_threadsafe(
                            lambda: asyncio.create_task(broadcast_entity_update(entity_type, entity_id))
                        )
                    logger.info(f"[TCP] Scheduled broadcast for {entity_type} with ID {entity_id}")
                
            response = "OK"
        except Exception as e:
            logger.error(f"[TCP] Error: {e}")
            response = "ERROR"
        self.request.sendall(response.encode('utf-8'))

# Include routers - API path prefix for all endpoints
API_PREFIX = "/api"

# Include websocket and streaming routers (these don't need the API prefix)
app.include_router(websocket_router, tags=["websocket"])
app.include_router(streaming_router, prefix="/stream", tags=["streaming"]) # RTSP 스트리밍 관련 라우터

# Include API routers
app.include_router(inventories.router, prefix=f"{API_PREFIX}/inventory", tags=["inventory"])
app.include_router(poses.router, prefix=f"{API_PREFIX}/pose6d", tags=["pose6d"])
app.include_router(robot.router, prefix=f"{API_PREFIX}/robots", tags=["robot"])
app.include_router(albabot.router, prefix=f"{API_PREFIX}/albabot", tags=["albabot"])
app.include_router(cookbot.router, prefix=f"{API_PREFIX}/cookbot", tags=["cookbot"])

# These routes were updated with prefix in their definition
app.include_router(auth.router, tags=["auth"])
app.include_router(users.router, tags=["users"])

# Continue with other routers
app.include_router(settings.router, prefix=f"{API_PREFIX}/settings", tags=["settings"])
app.include_router(customers.router, prefix=f"{API_PREFIX}/customers", tags=["customers"])
app.include_router(tables.router, prefix=f"{API_PREFIX}/tables", tags=["tables"])
app.include_router(kiosks.router, prefix=f"{API_PREFIX}/kiosks", tags=["kiosks"])
app.include_router(orders.router, prefix=f"{API_PREFIX}/orders", tags=["orders"])
app.include_router(menu.router, prefix=f"{API_PREFIX}/menu", tags=["menu"])
app.include_router(events.router, prefix=f"{API_PREFIX}/events", tags=["events"])
app.include_router(emergencies.router, prefix=f"{API_PREFIX}/emergencies", tags=["emergencies"])
app.include_router(video_streams.router, prefix=f"{API_PREFIX}/video-streams", tags=["video_streams"])

# --- TCP SERVER -----------------------------------------------------------
def start_tcp_server(host: str, port: int):
    server = socketserver.ThreadingTCPServer((host, port), RoboDineTCPHandler)
    logger.info(f"[TCP] Server listening on {host}:{port}")
    server.serve_forever()

# --- ENTRYPOINT -----------------------------------------------------------
def run_servers():
    import uvicorn
    uvicorn.run("robodine_service.backend.run:app", host="0.0.0.0", port=8000, reload=True)

if __name__ == "__main__":
    run_servers()
