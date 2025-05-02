# app/api/poses.py
from typing import Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from pydantic import BaseModel
from sqlalchemy.orm import Session

from app.core.db_config import get_db
from app.models.pose6d import Pose6D
from app.models.enums import EntityType

router = APIRouter(prefix="/pose6d", tags=["pose6d"])

class Pose6DData(BaseModel):
    entity_type: EntityType
    entity_id: int
    timestamp: Optional[datetime] = None
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    class Config:
        orm_mode = True

        

@router.post("/create", summary="Log a new Pose6D record")
def create_pose6d(
    data: Pose6DData,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    # set timestamp to now if not provided
    ts = data.timestamp or datetime.utcnow()

    # create new Pose6D record
    pose = Pose6D(
        entity_type=data.entity_type,
        entity_id=data.entity_id,
        timestamp=ts,
        x=data.x,
        y=data.y,
        z=data.z,
        roll=data.roll,
        pitch=data.pitch,
        yaw=data.yaw,
    )
    
    db.add(pose)
    db.commit()
    db.refresh(pose)

    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "pose6d",
        int(pose.entity_id)
    )

    return Pose6DData(
        entity_type=pose.entity_type,
        entity_id=pose.entity_id,
        timestamp=pose.timestamp,
        x=pose.x,
        y=pose.y,
        z=pose.z,
        roll=pose.roll,
        pitch=pose.pitch,
        yaw=pose.yaw,
    )


# Optional endpoint for querying recent poses
@router.get("/recent/{entity_id}", summary="Get recent poses for an entity")
def get_recent_poses(
    entity_id: int,
    limit: int = 20,
    db: Session = Depends(get_db)
):
    poses = (
        db.query(Pose6D)
        .filter(Pose6D.entity_id == entity_id)
        .order_by(Pose6D.timestamp.desc())
        .limit(limit)
        .all()
    )
    if not poses:
        raise HTTPException(status_code=404, detail="No poses found for this entity")
    return poses
