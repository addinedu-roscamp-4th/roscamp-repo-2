from app.models import CleaningTask
from app.core.db_config import SessionLocal
from sqlalchemy.orm import Session
from datetime import datetime

def start_cleaning_task(robot_id: str, area: str):
    # 청소 작업 시작
    db: Session = SessionLocal()
    task = CleaningTask(robot_id=robot_id, area=area, status="진행 중", started_at=datetime.now())
    db.add(task)
    db.commit()
    db.close()
    print("✅ 청소 작업이 시작되었습니다.")
