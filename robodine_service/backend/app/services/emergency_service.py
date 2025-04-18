from app.models import Emergency
from app.core.db_config import SessionLocal
from sqlalchemy.orm import Session

def report_emergency(emergency_data: dict):
    # 비상 상황 신고
    db: Session = SessionLocal()
    emergency = Emergency(**emergency_data)  # 비상 데이터로 객체 생성
    db.add(emergency)
    db.commit()
    db.close()
    print("✅ 비상 상황이 데이터베이스에 등록되었습니다!")
