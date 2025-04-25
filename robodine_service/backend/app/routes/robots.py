from fastapi import APIRouter
from app.models import Robot
from app.core.db_config import SessionLocal
from sqlalchemy.orm import Session
from pydantic import BaseModel
from datetime import datetime
from typing import Optional
from app.models.enums import EntityType, RobotStatus

router = APIRouter()

class RobotData(BaseModel):
    robot_id: Optional[str] = None
    type: Optional[EntityType] = None
    mac_address: Optional[str] = None
    ip_address: Optional[str] = None

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
