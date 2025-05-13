# robodine_service/backend/app/core/utils.py
from typing import Dict, Any, Optional, Type
from sqlmodel import Session, SQLModel
from datetime import datetime
from fastapi import HTTPException, status
import base64, uuid, os
import json

from app.models.robot import Robot
from app.models.pose6d import Pose6D
from app.models.jointangle import JointAngle
from app.models.albabot import Albabot
from app.models.cookbot import Cookbot
from app.models.face_recognition import FaceRecognition
from app.core.db_config import get_db
from app.models.table import Table
from app.models.chat import Chat
from app.models.enums import TableStatus, ChatStatus
from app.core.config import IMAGES_DIR


def dispatch_payload(session: Session, data: Dict[str, Any]) -> Dict[str, Any]:
    print(f"Dispatching payload: {data}")
    msg_type = data.get("msg_type")
    if msg_type == "Albabot":
        return _handle_albabot(session, data)
    elif msg_type == "Cookbot":
        return _handle_cookbot(session, data)
    elif msg_type == "Ingredient":
        return _handle_ingredient(session, data)
    elif msg_type == "Face":
        return _face_recognition(session, data)
    elif msg_type == "Chatbot":
        return _chatbot(session, data)
    else:
        raise ValueError(f"Unknown msg_type: {msg_type}")

def _get_last_status(session: Session, robot_id: int, robot_type) -> Any:
    """
    가장 최근에 저장된, null이 아닌 status를 꺼내 옵니다.
    """
    instance = session.get(robot_type, robot_id)
    return instance.status if instance and instance.status is not None else None

def _handle_albabot(session: Session, data: Dict[str, Any]):
    robot_id = data["robot_id"]
    # # 1) 기존 status를 그대로 가져오기
    # last_status = _get_last_status(session, robot_id, Albabot)
    # # 2) Robot 테이블에 status만 이전 값 그대로 추가해서 행 추가
    # 
    session.add(Albabot(
        robot_id=robot_id,
        # status=last_status,
        status=data["status"],
        battery_level=data["battery_level"],
    ))
    # 3) 3가지 Pose6D 레코드 추가
    for entity, prefix in [("PINKY", "pinky"), ("GLOBAL", "global"), ("WORLD", "world")]:
        session.add(Pose6D(
            entity_id=robot_id,
            entity_type=entity,
            timestamp=datetime.now(),
            x=data[f"{prefix}_x"],
            y=data[f"{prefix}_y"],
            z=data[f"{prefix}_z"],
            roll=data[f"{prefix}_roll"],
            pitch=data[f"{prefix}_pitch"],
            yaw=data[f"{prefix}_yaw"],
        ))
    return {"affected_entity": {"type": "albabot", "id": robot_id}}

def _handle_cookbot(session: Session, data: Dict[str, Any]):
    robot_id = data["robot_id"]
    # # 1) 기존 status를 그대로 가져오기
    # last_status = _get_last_status(session, robot_id)
    # 2) Robot 테이블에 status만 이전 값 그대로 업데이트
    session.add(Cookbot(
        robot_id=robot_id,
        status=data["status"],
        # status=last_status,
    ))
    # 3) Endpoint Pose6D 레코드 추가
    session.add(Pose6D(
        entity_id=robot_id,
        entity_type="COOKBOT",
        timestamp=datetime.now(),
        x=data["endpoint_x"],
        y=data["endpoint_y"],
        z=data["endpoint_z"],
        roll=data["endpoint_roll"],
        pitch=data["endpoint_pitch"],
        yaw=data["endpoint_yaw"],
    ))
    # 4) JointAngle 레코드 추가
    session.add(JointAngle(
        robot_id=robot_id,
        timestamp=datetime.now(),
        joint_1=data["angle_1"],
        joint_2=data["angle_2"],
        joint_3=data["angle_3"],
        joint_4=data["angle_4"],
        joint_5=data["angle_5"],
        joint_6=data["angle_6"],
    ))
    return {"affected_entity": {"type": "cookbot", "id": robot_id}}


def _handle_ingredient(session: Session, data: Dict[str, Any]):
    session.add(Pose6D(
        entity_id=data["ingredient_id"],
        entity_type="INVENTORY",
        timestamp=datetime.now(),
        x=data["x"],
        y=data["y"],
        z=data["z"],
        roll=data["roll"],
        pitch=data["pitch"],
        yaw=data["yaw"],
    ))
    return {"affected_entity": {"type": "inventory", "id": data["ingredient_id"]}}

def _face_recognition(session: Session, data: Dict[str, Any]):
    session.add(FaceRecognition(
        table_id=data["table_id"],
        timestamp=datetime.now(),
        history=data["history"],
        nowdetected=data["nowdetected"],
        reliability=data["reliability"],
        exist=data["exist"],
    ))

    db = session

    # exist가 1인 경우에 테이블의 상태 변경
    if data["exist"] == 1:
        table = db.query(Table).filter(Table.table_id == data["table_id"]).first()

        if table:
            table.status = TableStatus.OCCUPIED
            db.add(table)
            db.commit()
            db.refresh(table)
        else:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Table with ID {data['table_id']} not found"
            )
    
    return {"affected_entity": {"type": "face", "id": data["table_id"]}}

def _chatbot(session: Session, data: Dict[str, Any]):
    chat = session.query(Chat).filter(Chat.id == data["msg_id"]).first()
    if not chat:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND,
                            detail=f"Chat with ID {data['msg_id']} not found")

    # 공통 필드 업데이트
    chat.question   = data["question"]
    chat.robot_id   = data["robot_id"]
    chat.robot_task = data["robot_task"]
    chat.status     = ChatStatus.COMPLETED
    
    if data["robot_task"] == "TAKE_PICTURE":
        # -- 이미지 저장 --
        img_bytes = base64.b64decode(data["response_image"])
        fname     = f"{uuid.uuid4().hex}.png"
        out_path  = os.path.join(IMAGES_DIR, fname)
        with open(out_path, "wb") as wf:
            wf.write(img_bytes)

        # -- 텍스트 + 이미지 URL을 JSON 문자열로 answer에 저장 --
        answer_payload = {
            "text": data["response_text"],
            "image": f"/images/{fname}"
        }
        chat.answer = json.dumps(answer_payload, ensure_ascii=False)
    else:
        # 일반 텍스트 응답인 경우
        chat.answer = data["response_text"]

    session.add(chat)
    session.commit()
    session.refresh(chat)
    return {"affected_entity": {"type": "chat", "id": chat.id}}

# 채팅 업데이트 전송 함수
async def broadcast_chat_update(chat_id: int, session: Session = None):
    """
    채팅 ID에 대한 업데이트를 웹소켓으로 브로드캐스팅
    
    Args:
        chat_id: 업데이트할 채팅 메시지 ID
        session: 데이터베이스 세션 (없으면 새로 생성)
    """
    try:
        from run import broadcast_entity_update
        await broadcast_entity_update("chat", chat_id)
    except Exception as e:
        import logging
        logger = logging.getLogger(__name__)
        logger.error(f"채팅 업데이트 브로드캐스팅 중 오류: {str(e)}")