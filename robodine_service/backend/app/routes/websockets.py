# robodine_service/backend/app/routes/websockets.py

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException
import logging
import json
import os
from typing import Dict, List, Optional, Any, Literal, Union
from pydantic import BaseModel
import asyncio

logger = logging.getLogger(__name__)

router = APIRouter()

# 웹소켓 설정 - 환경 변수나 설정에서 읽어옴
WS_MAX_CONNECTIONS = int(os.environ.get("WS_MAX_CONNECTIONS", "100"))
WS_PING_INTERVAL = int(os.environ.get("WS_PING_INTERVAL", "30"))  # 초 단위

# 웹소켓 메시지 프로토콜 정의
class WSMessage(BaseModel):
    type: Literal["update", "error", "ping", "pong", "shutdown"]
    topic: Literal["robots", "tables", "events", "orders", "status", "systemlogs", "customers", "inventory", "video_streams", "notifications", "commands"]
    data: Any

# 연결 관리자 클래스
class ConnectionManager:
    def __init__(self):
        # 토픽별 활성 연결 저장
        self.active_connections: Dict[str, List[WebSocket]] = {
            "robots": [],
            "tables": [],
            "events": [],
            "orders": [],
            "status": [],
            "systemlogs": [],
            "customers": [],
            "inventory": [],
            "video_streams": [],
            "notifications": [],
            "commands": []
        }
        self.shutting_down = False
        self.max_connections_per_topic = WS_MAX_CONNECTIONS  # 환경 변수에서 가져온 값

    async def connect(self, websocket: WebSocket, topic: str):
        """웹소켓 연결 수락 및 토픽에 등록"""
        # 종료 중이면 연결 거부
        if self.shutting_down:
            await websocket.close(code=1001, reason="Server is shutting down")
            return False
            
        # 토픽별 최대 연결 수 제한
        if len(self.active_connections.get(topic, [])) >= self.max_connections_per_topic:
            logger.warning(f"Maximum connections reached for topic: {topic}")
            await websocket.close(code=1008, reason="Too many connections")
            return False
            
        await websocket.accept()
        if topic not in self.active_connections:
            self.active_connections[topic] = []
            
        self.active_connections[topic].append(websocket)
        logger.info(f"New WebSocket connection established for topic: {topic}. Total connections: {len(self.active_connections[topic])}")
        return True

    def disconnect(self, websocket: WebSocket, topic: str):
        """웹소켓 연결 해제 및 토픽에서 제거"""
        if topic in self.active_connections and websocket in self.active_connections[topic]:
            self.active_connections[topic].remove(websocket)
            logger.info(f"WebSocket connection closed for topic: {topic}. Remaining connections: {len(self.active_connections[topic])}")

    async def send_ping_to_all(self):
        """모든 연결에 정기적으로 핑 메시지 전송"""
        while not self.shutting_down:
            try:
                ping_message = {
                    "type": "ping",
                    "topic": "status",
                    "data": {"timestamp": asyncio.get_event_loop().time()}
                }
                ping_str = json.dumps(ping_message)
                
                for topic, connections in self.active_connections.items():
                    for conn in connections[:]:  # 복사본으로 반복
                        try:
                            await conn.send_text(ping_str)
                        except Exception as e:
                            logger.error(f"Error sending ping to client on topic {topic}: {e}")
                            self.disconnect(conn, topic)
                            
                # 핑 간격만큼 대기
                await asyncio.sleep(WS_PING_INTERVAL)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in ping loop: {e}")
                await asyncio.sleep(WS_PING_INTERVAL)

    async def broadcast(self, message_data: dict, topic: str):
        """특정 토픽에 등록된 모든 클라이언트에 메시지 브로드캐스팅"""
        if topic not in self.active_connections:
            return
        
        message_str = json.dumps(message_data)
        failed_connections = []
        
        for connection in self.active_connections[topic]:
            try:
                await connection.send_text(message_str)
            except Exception as e:
                logger.error(f"Error broadcasting to client: {str(e)}")
                failed_connections.append(connection)
                
        # 실패한 연결은 나중에 한꺼번에 제거
        for connection in failed_connections:
            self.disconnect(connection, topic)
    
    async def broadcast_update(self, 
                             data: Any, 
                             topic: Literal["robots", "tables", "events", "orders", "systemlogs", "customers", "inventory", "notifications", "commands"]):
        """데이터 객체를 JSON으로 직렬화하여 특정 토픽에 브로드캐스팅"""
        message = {
            "type": "update",
            "topic": topic,
            "data": data
        }
        await self.broadcast(message, topic)
        
    async def shutdown(self):
        """모든 웹소켓 연결을 깔끔하게 닫는 함수"""
        self.shutting_down = True
        
        # 모든 클라이언트에 종료 메시지 전송
        shutdown_message = {
            "type": "shutdown",
            "topic": "status",
            "data": {"reason": "Server is shutting down"}
        }
        
        shutdown_str = json.dumps(shutdown_message)
        for topic, connections in self.active_connections.items():
            for conn in connections[:]:  # 복사본으로 반복
                try:
                    await conn.send_text(shutdown_str)
                    await conn.close(code=1001)
                except Exception as e:
                    logger.error(f"Error closing client connection: {e}")
                
        # 모든 연결 목록 초기화
        for topic in self.active_connections:
            self.active_connections[topic] = []
            
        logger.info("All WebSocket connections have been closed")

manager = ConnectionManager()

# 연결 관리자 핑 태스크
ping_task = None

# 통합된 웹소켓 엔드포인트
@router.websocket("/ws/{topic}")
async def websocket_topic_endpoint(websocket: WebSocket, topic: str):
    """단일 통합 웹소켓 엔드포인트 - 토픽은 URL 경로 파라미터로 지정"""
    # 지원되는 토픽 확인
    valid_topics = ["robots", "tables", "events", "orders", "status", "systemlogs", "customers", "inventory", "video_streams", "notifications", "commands"]
    if topic not in valid_topics:
        logger.warning(f"Client attempted to connect to invalid topic: {topic}")
        await websocket.close(code=1003)  # 1003 = Unsupported data
        return
        
    # 연결 처리
    connected = await manager.connect(websocket, topic)
    if not connected:
        return  # 연결 실패 시 즉시 반환
        
    try:      
        from run import broadcast_entity_update 
        # 초기 브로드캐스트: 로봇 연결 시 전체 상태 즉시 전송
        if topic == "robots":
            try:
                await broadcast_entity_update("robot", None)
            except Exception as e:
                logger.error(f"Initial robot broadcast failed: {e}")
        
        # 초기 브로드캐스트: 재고 연결 시 전체 재고 즉시 전송
        elif topic == "inventory":
            try:
                await broadcast_entity_update("inventory", None)
            except Exception as e:
                logger.error(f"Initial inventory broadcast failed: {e}")

        # 초기 브로드캐스트: 알림 연결 시 전체 알림 즉시 전송
        elif topic == "notifications":
            try:
                await broadcast_entity_update("notification", None)
            except Exception as e:
                logger.error(f"Initial notifications broadcast failed: {e}")
                
        # 초기 브로드캐스트: 명령어 로그 연결 시 명령어 목록 즉시 전송
        elif topic == "commands":
            try:
                await broadcast_entity_update("command", None)
            except Exception as e:
                logger.error(f"Initial commands broadcast failed: {e}")

        logger.info(f"{topic.capitalize()} connection established from {websocket.client.host}")
        
        # 클라이언트에 연결 확인 메시지 전송
        await websocket.send_text(json.dumps({
            "type": "update",
            "topic": topic,
            "data": {"status": "connected", "message": f"Connected to {topic} updates"}
        }))
        
        # 연결 유지 및 메시지 수신 루프
        while True:
            try:
                data = await websocket.receive_text()
                client_message = json.loads(data)
                # logger.info(f"Received message from client on topic {topic}: {data}")
                
                # 핑-퐁 처리
                if client_message.get("type") == "ping":
                    await websocket.send_text(json.dumps({
                        "type": "pong", 
                        "topic": topic,
                        "data": {"timestamp": client_message.get("data", {}).get("timestamp")}
                    }))
                    
            except WebSocketDisconnect:
                manager.disconnect(websocket, topic)
                logger.info(f"Client disconnected from {topic}")
                break
                
    except Exception as e:
        logger.error(f"WebSocket error on topic {topic}: {str(e)}")
        manager.disconnect(websocket, topic)

# 이전 엔드포인트 유지 (하위 호환성)
@router.websocket("/ws/status")
async def legacy_websocket_status_endpoint(websocket: WebSocket):
    """하위 호환성을 위한 이전 status 엔드포인트"""
    await websocket_topic_endpoint(websocket, "status")

# 서버 시작 시 핑 태스크 등록
@router.on_event("startup")
async def start_ping_task():
    global ping_task
    ping_task = asyncio.create_task(manager.send_ping_to_all())
    logger.info("Started WebSocket ping task")

# 서버 종료 시 정리
@router.on_event("shutdown")
async def stop_ping_task():
    global ping_task
    if ping_task:
        ping_task.cancel()
        try:
            await ping_task
        except asyncio.CancelledError:
            logger.info("WebSocket ping task cancelled")
    
    # 모든 웹소켓 연결 깔끔하게 종료
    await manager.shutdown()

# 서버 측에서 사용할 데이터 전송 함수들
async def broadcast_robots_update(robots_data):
    """로봇 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(robots_data, "robots")

async def broadcast_tables_update(tables_data):
    """테이블 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(tables_data, "tables")

async def broadcast_events_update(events_data):
    """이벤트 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(events_data, "events")

async def broadcast_orders_update(orders_data):
    """주문 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(orders_data, "orders")

async def broadcast_customers_update(customers_data):
    """고객 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(customers_data, "customers")

async def broadcast_systemlogs_update(logs_data):
    """시스템 로그 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(logs_data, "systemlogs")

async def broadcast_notifications_update(notifications_data):
    """알림 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(notifications_data, "notifications")

async def broadcast_commands_update(commands_data):
    """명령어 로그 데이터 업데이트를 브로드캐스팅"""
    await manager.broadcast_update(commands_data, "commands")
