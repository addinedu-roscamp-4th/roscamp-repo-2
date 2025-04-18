from fastapi import APIRouter
from app.models import Robot
from app.core.db_config import SessionLocal
from sqlalchemy.orm import Session
from pydantic import BaseModel
from datetime import datetime
from typing import Optional

router = APIRouter()

class RobotData(BaseModel):
    id: str  # Only ID is required
    mac_address: Optional[str] = None
    ip_address: Optional[str] = None
    status: Optional[str] = None
    location: Optional[str] = None
    battery_level: Optional[int] = None
    timestamp: Optional[datetime] = None  # timestamp optional

@router.post("/update_robot")
def update_robot_status(robot_data: RobotData):
    db: Session = SessionLocal()
    
    # Check if robot already exists in the database
    robot = db.query(Robot).filter(Robot.id == robot_data.id).first()
    
    if robot:
        # Update only the fields that are provided
        for key, value in robot_data.dict(exclude_unset=True).items():
            if value is not None:
                setattr(robot, key, value)
    else:
        # Create a new robot with the provided fields
        robot = Robot(**robot_data.dict(exclude_unset=True))
    
    db.add(robot)
    db.commit()
    db.close()
    return {"message": "Robot data processed successfully!"}
