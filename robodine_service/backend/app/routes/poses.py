# app/api/poses.py
from typing import Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException
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

    return {
        "message": "Pose6D logged successfully",
        "pose6d": pose,
    }

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
