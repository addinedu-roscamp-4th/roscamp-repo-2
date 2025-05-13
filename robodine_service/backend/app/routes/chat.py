# robodine_service/backend/app/routes/chat.py

import logging
import asyncio
import socket
import json
import uuid
from datetime import datetime
from typing import Dict, Any, List, Optional

from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException, status
from fastapi.responses import JSONResponse
from sqlmodel import Session, select, func
from pydantic import BaseModel

from app.core.db_config import get_db, SessionLocal
from app.models.chat import Chat
from app.models.user import User
from app.core.utils import broadcast_chat_update
from app.routes.auth import get_current_user
from app.models.enums import ChatStatus

# 로깅 설정
logger = logging.getLogger(__name__)
router = APIRouter()



# 채팅 TCP 서버 설정
CHAT_SERVER_HOST = "192.168.0.135"  # 로컬 테스트용, 실제 서버 IP로 변경 필요
CHAT_SERVER_PORT = 8001  # 채팅 서버가 사용하는 포트

# 요청 메시지 스키마
class ChatRequest(BaseModel):
    message: str

# 응답 메시지 스키마
class ChatResponse(BaseModel):
    id: int
    question: str
    answer: Optional[str]
    timestamp: datetime
    status: str

# 페이징 정보 스키마
class PaginationInfo(BaseModel):
    total: int
    page: int
    per_page: int
    total_pages: int

# 채팅 히스토리 응답 스키마
class ChatHistoryResponse(BaseModel):
    messages: List[ChatResponse]
    pagination: PaginationInfo

# Configuration
MAX_RETRIES = 3            # 총 재시도 횟수
INITIAL_DELAY = 3          # 첫 번째 재시도 전 대기 시간(초)
BACKOFF_FACTOR = 2         # 지수 백오프 계수

async def send_to_chat_server(message: Dict[str, Any]) -> Dict[str, Any]:
    """채팅 메시지를 외부 서버로 TCP를 통해 전송하고, ACK 수신만 처리"""
    attempt = 0
    delay = INITIAL_DELAY

    while attempt < MAX_RETRIES:
        attempt += 1
        client_socket = None
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(10)
            client_socket.connect((CHAT_SERVER_HOST, CHAT_SERVER_PORT))

            payload = json.dumps(message).encode('utf-8')
            header = len(payload).to_bytes(4, byteorder='big')
            client_socket.sendall(header + payload)

            # ACK 수신
            hdr = client_socket.recv(4)
            if not hdr:
                raise ConnectionError("빈 ACK 헤더를 받았습니다.")
            size = int.from_bytes(hdr, byteorder='big')
            data = b''
            while len(data) < size:
                chunk = client_socket.recv(min(size - len(data), 4096))
                if not chunk:
                    raise ConnectionError("ACK 수신 중 연결이 끊어졌습니다.")
                data += chunk

            ack_text = data.decode('utf-8')
            logger.info(f"채팅 서버 ACK: {ack_text}")
            return {"msg_id": message.get("msg_id"), "response_text": ack_text, "status": "ACK"}

        except (socket.timeout, ConnectionRefusedError, ConnectionError) as e:
            logger.error(f"시도 {attempt}/{MAX_RETRIES} - ACK 수신 오류: {e}")
            if attempt < MAX_RETRIES:
                logger.info(f"{delay}초 후 재시도합니다...")
                await asyncio.sleep(delay)
                delay *= BACKOFF_FACTOR
                continue
            return {"msg_id": message.get("msg_id"), "response_text": "ACK 수신 실패", "status": "error"}
        finally:
            if client_socket:
                try:
                    client_socket.close()
                except:
                    pass

async def process_chat_message(
    chat_id: int,
    message: str,
):
    """채팅 메시지를 전송하고, ACK 수신 시 상태만 업데이트"""
    session = SessionLocal()

    # 상태를 PENDING으로 유지 (전송 완료 후에도 응답 대기)
    chat = session.get(Chat, chat_id)
    chat.status = "PENDING"
    session.add(chat)
    session.commit()
    session.refresh(chat)
    from run import broadcast_entity_update
    asyncio.create_task(broadcast_entity_update("chat", chat_id))

    response = await send_to_chat_server({
        "msg_id": chat_id,
        "question": message,
        "timestamp": datetime.now().isoformat()
    })

    if response.get("status") == "error":
        chat = session.get(Chat, chat_id)
        chat.answer = "채팅 서버에 연결할 수 없습니다. 잠시 후 다시 시도해주세요."
        chat.status = "ERROR"
        session.add(chat)
        session.commit()
        session.refresh(chat)
        asyncio.create_task(broadcast_entity_update("chat", chat_id))

    session.close()



# 채팅 엔드포인트    session: Session = None

@router.post("/send", response_model=ChatResponse)
async def send_chat(
    chat_request: ChatRequest,
    background_tasks: BackgroundTasks,
    session: Session = Depends(get_db)
):
    """새 채팅 메시지 전송"""
    try:
        # 새 채팅 레코드 생성
        new_chat = Chat(
            question=chat_request.message,
            status="PENDING",
            timestamp=datetime.now()
        )
        
        session.add(new_chat)
        session.commit()
        session.refresh(new_chat)
        
        # 백그라운드에서 메시지 처리
        background_tasks.add_task(
            process_chat_message,
            new_chat.id,
            chat_request.message,
        )

        from run import broadcast_entity_update
        # REST API 호출 시 웹소켓 브로드캐스트 트리거
        background_tasks.add_task(
            broadcast_entity_update,
            "chat",
            None
        )
        
        return JSONResponse(
            status_code=status.HTTP_202_ACCEPTED,
            content={
                "id": new_chat.id,
                "question": new_chat.question,
                "answer": None,
                "timestamp": new_chat.timestamp.isoformat(),
                "status": "PENDING"
            }
        )
        
    except Exception as e:
        logger.error(f"채팅 전송 중 오류: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="메시지 처리 중 오류가 발생했습니다."
        )

# 채팅 기록 조회
@router.get("/history", response_model=ChatHistoryResponse)
async def get_chat_history(
    page: int = 1,
    per_page: int = 20,
    search: Optional[str] = None,
    session: Session = Depends(get_db),
):
    """사용자 채팅 기록 조회 (페이징 및 검색 지원)"""
    try:
        # 기본 쿼리 구성
        query = select(Chat)

        # 검색어가 있는 경우 검색 조건 추가
        if search and search.strip():
            search_term = f"%{search.strip()}%"
            query = query.where(
                (Chat.question.ilike(search_term)) |
                (Chat.answer.ilike(search_term))
            )

        # 전체 개수 조회
        total_count = session.execute(
            select(func.count()).select_from(query.subquery())
        ).scalar() or 0

        # 페이징 적용
        offset = (page - 1) * per_page
        result = session.execute(
            query
            .order_by(Chat.timestamp.desc())
            .offset(offset)
            .limit(per_page)
        )
        chats = result.scalars().all()

        # 응답 데이터 구성
        return ChatHistoryResponse(
            messages=[
                ChatResponse(
                    id=chat.id,
                    question=chat.question,
                    answer=chat.answer,
                    timestamp=chat.timestamp,
                    status=chat.status or ""     # None인 경우 빈 문자열로 대체
                )
                for chat in chats
            ],
            pagination=PaginationInfo(
                total=total_count,
                page=page,
                per_page=per_page,
                total_pages=(total_count + per_page - 1) // per_page
            )
        )

    except Exception as e:
        logger.error(f"채팅 기록 조회 중 오류: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"채팅 기록을 불러오는 중 오류가 발생했습니다: {str(e)}"
        )


# 채팅 상세 조회
@router.get("/{chat_id}", response_model=ChatResponse)
async def get_chat_detail(
    chat_id: int,
    session: Session = Depends(get_db)
):
    """채팅 메시지 상세 조회"""
    chat = session.get(Chat, chat_id)
    
    if not chat:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="채팅 메시지를 찾을 수 없습니다."
        )
        
    return {
        "id": chat.id,
        "question": chat.question,
        "answer": chat.answer,
        "timestamp": chat.timestamp,
        "status": chat.status
    } 