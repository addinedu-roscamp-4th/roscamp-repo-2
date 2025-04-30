# robodine_service/backend/app/core/utils.py
from typing import Dict, Any, Optional, Type
from sqlmodel import Session, SQLModel
from datetime import datetime

from app.models.robot import Robot
from app.models.pose6d import Pose6D
from app.models.jointangle import JointAngle
from app.models.albabot import Albabot
from app.models.cookbot import Cookbot


def dispatch_payload(session: Session, data: Dict[str, Any]) -> None:
    msg_type = data.get("msg_type")
    if msg_type == "Albabot":
        _handle_albabot(session, data)
    elif msg_type == "Cookbot":
        _handle_cookbot(session, data)
    elif msg_type == "Ingredient":
        _handle_ingredient(session, data)
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
