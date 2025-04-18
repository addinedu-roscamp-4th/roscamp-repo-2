from app.models import Robot
from app.core.db_config import SessionLocal
from sqlalchemy.orm import Session
import json

def process_ros_data(ros_data: str):
    # ROS2로부터 받은 데이터 처리 (예: JSON 형식)
    data = json.loads(ros_data)
    
    # 데이터베이스에 로봇 상태 업데이트
    db: Session = SessionLocal()
    robot = Robot(**data)  # 로봇 데이터로 객체 생성
    db.add(robot)
    db.commit()
    db.close()

    print("✅ 로봇 데이터가 데이터베이스에 저장되었습니다!")
