# robodine_service/backend/run.py

import logging
import threading
import socketserver
import json
from datetime import datetime
from contextlib import asynccontextmanager

from fastapi import FastAPI
from starlette.middleware.cors import CORSMiddleware
from sqlmodel import SQLModel

from app.routes import (
    poses, websockets, streaming, inventories,
    robot, albabot, cookbot, auth, users, settings, customers,
    tables, kiosks, orders, menu, events, emergencies, 
    video_streams
)
from app.routes.websockets import router as websocket_router
from app.routes.streaming import router as streaming_router
from app.core.db_config import engine
from app.core.database import get_session
from app.models.robot import Robot
from app.core.utils import dispatch_payload

from app.services.streaming_service import get_stream_urls, add_stream_url

# 로깅 설정
logger = logging.getLogger("robodine.run")
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info("Starting up application...")
    SQLModel.metadata.create_all(bind=engine)
    logger.info("Database tables created successfully")

    # TCP 서버
    tcp_thread = threading.Thread(target=start_tcp_server, args=("0.0.0.0", 8001), daemon=True)
    tcp_thread.start()

    # 초기 RTSP 녹화 시작 (config로 미리 등록된 URL이 있으면)
    for url in get_stream_urls():
        # 이미 서비스 모듈에서 쓰레드를 띄우도록 설계했으니 단순 get만
        pass

    yield
    logger.info("Shutting down application...")

app = FastAPI(lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# Include routers
app.include_router(websocket_router, tags=["websocket"])
app.include_router(inventories.router, prefix="/inventory", tags=["inventory"])
app.include_router(streaming.router, prefix="/stream", tags=["streaming"]) # RTSP 스트리밍 관련 라우터
app.include_router(poses.router, prefix="/pose6d", tags=["pose6d"])

# New routers
app.include_router(robot.router, prefix="/robot", tags=["robot"])
app.include_router(albabot.router, prefix="/albabot", tags=["albabot"])
app.include_router(cookbot.router, prefix="/cookbot", tags=["cookbot"])
app.include_router(auth.router, prefix="/auth", tags=["auth"])
app.include_router(users.router, prefix="/users", tags=["users"])
app.include_router(settings.router, prefix="/settings", tags=["settings"])
app.include_router(customers.router, prefix="/customers", tags=["customers"])
app.include_router(tables.router, prefix="/tables", tags=["tables"])
app.include_router(kiosks.router, prefix="/kiosks", tags=["kiosks"])
app.include_router(orders.router, prefix="/orders", tags=["orders"])
app.include_router(menu.router, prefix="/menu", tags=["menu"])
app.include_router(events.router, prefix="/events", tags=["events"])
app.include_router(emergencies.router, prefix="/emergencies", tags=["emergencies"])
app.include_router(video_streams.router, prefix="/video-streams", tags=["video_streams"])

# --- TCP SERVER -----------------------------------------------------------
class RoboDineTCPHandler(socketserver.BaseRequestHandler):
    def handle(self):
        raw = self.request.recv(1024).strip()
        logger.info(f"[TCP] Received: {raw}")
        try:
            payload = json.loads(raw.decode('utf-8'))
            with get_session() as session:
                # msg_type 에 따라 분기 처리
                dispatch_payload(session, payload)
                session.commit()
            response = "OK"
        except Exception as e:
            logger.error(f"[TCP] Error: {e}")
            response = "ERROR"
        self.request.sendall(response.encode('utf-8'))

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
