import sys
import os

# 현재 파일 위치에서 상위 폴더(app 폴더의 상위) 경로 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'app')))

from sqlmodel import SQLModel
from app.models import Robot, Customer, Order, Inventory, Event, Table, WaitingList, CleaningTask, Emergency, VideoStream, RobotCommand, AdminSettings
from app.core.db_config import engine

def create_db_and_tables():
    # 모든 테이블 생성
    SQLModel.metadata.create_all(bind=engine)
    print("✅ 모든 테이블이 성공적으로 생성되었습니다!")

if __name__ == "__main__":
    create_db_and_tables()
