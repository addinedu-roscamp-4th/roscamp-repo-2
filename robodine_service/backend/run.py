# robodine_service/backend/run.py

import logging
import threading
import socketserver
import json
from datetime import datetime
import subprocess
import os

from fastapi import FastAPI
from starlette.middleware.cors import CORSMiddleware
from sqlmodel import SQLModel

from app.routes import robots, inventory, control
from app.routes.websockets import router as websocket_router
from app.core.db_config import engine
from app.core.database import get_session
from app.models.robot import Robot
from app.core.utils import update_model

# 로깅 설정
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# FastAPI 앱 및 lifespan 정의
from contextlib import asynccontextmanager

@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info("Starting up application...")

    # DB 테이블 생성
    SQLModel.metadata.create_all(bind=engine)
    logger.info("Database tables created successfully")

    # TCP 서버 스레드 시작
    tcp_thread = threading.Thread(target=start_tcp_server, args=("0.0.0.0", 8001), daemon=True)
    tcp_thread.start()

    # RTSP 서버 스레드 시작
    rtsp_thread = threading.Thread(target=start_rtsp_server, daemon=True)
    rtsp_thread.start()

    yield  # 앱이 실행되는 동안 지속

app = FastAPI(lifespan=lifespan)

# CORS 미들웨어 추가
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"]
)

# 라우트 포함
app.include_router(robots.router, prefix="/robots", tags=["robots"])
app.include_router(inventory.router, prefix="/inventory", tags=["inventory"])
app.include_router(control.router, prefix="/control", tags=["control"])
app.include_router(websocket_router, tags=["websocket"])

# TCP 서버 핸들러 클래스 정의
class RoboDineTCPHandler(socketserver.BaseRequestHandler):
    def handle(self):
        data = self.request.recv(1024).strip()
        logger.info(f"[TCP 서버] 수신 데이터: {data}")

        try:
            json_data = json.loads(data.decode('utf-8'))

            # ID가 필수값
            if "id" not in json_data:
                raise ValueError("ID field is required")

            robot_id = json_data["id"]

            with get_session() as session:
                # 새 유틸리티 함수를 사용하여 로봇 모델 업데이트
                update_model(session, Robot, robot_id, json_data)
                session.commit()

            logger.info("[TCP 서버] 데이터 DB 저장 성공")
            response = "OK"

        except Exception as e:
            logger.error(f"[TCP 서버] 데이터 처리 오류: {e}")
            response = "ERROR"

        self.request.sendall(response.encode('utf-8'))


# TCP 서버 실행 함수 정의
def start_tcp_server(host="0.0.0.0", port=8001):
    with socketserver.ThreadingTCPServer((host, port), RoboDineTCPHandler) as server:
        logger.info(f"[TCP 서버] {host}:{port}에서 서버 시작됨")
        server.serve_forever()

# RTSP 서버 실행 함수 정의 (Docker로 rtsp-simple-server 활용)
def start_rtsp_server():
    command = [
        "docker", "run", "--rm", "-it",
        "-p", "8554:8554",
        "-p", "8050:8050/udp",
        "-p", "8051:8051/udp",
        "-v", f"{os.getcwd()}/rtsp-simple-server.yml:/mediamtx.yml",
        "aler9/rtsp-simple-server"
    ]
    logger.info("[RTSP 서버] Docker RTSP 서버를 시작합니다.")
    subprocess.run(command)
