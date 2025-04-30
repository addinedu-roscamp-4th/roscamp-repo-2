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

from app.routes import robots, control, poses, websockets, streaming, inventories
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
app.include_router(websocket_router, tags=["websocket"])
app.include_router(robots.router, prefix="/robots", tags=["robots"])
app.include_router(inventories.router, prefix="/inventories", tags=["inventories"])
app.include_router(control.router, prefix="/control", tags=["control"])
app.include_router(streaming.router, prefix="/stream", tags=["streaming"]) # RTSP 스트리밍 관련 라우터
app.include_router(poses.router, prefix="/pose6d", tags=["pose6d"])

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
