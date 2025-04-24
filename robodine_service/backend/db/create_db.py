import sys
import os

# 현재 파일 위치에서 상위 폴더(app 폴더의 상위) 경로 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'app')))

from sqlmodel import SQLModel
from app.models import Robot, Customer, Order, Inventory, Event, Table, WaitingList, CleaningTask, Emergency, VideoStream, RobotCommand, AdminSettings
from app.core.db_config import engine
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy import text

# SQLAlchemy 엔진 및 세션 생성
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# 기존 Enum 타입을 자동으로 삭제하는 함수
def drop_all_enums():
    with engine.connect() as connection:
        # PostgreSQL에서 모든 ENUM 타입 목록을 조회하는 쿼리
        enums_query = """
        SELECT n.nspname as enum_schema,
               t.typname as enum_name
        FROM pg_type t
        JOIN pg_catalog.pg_enum e ON e.enumtypid = t.oid
        JOIN pg_catalog.pg_namespace n ON n.oid = t.typnamespace
        WHERE t.typtype = 'e'
        ORDER BY enum_schema, enum_name;
        """
        
        result = connection.execute(text(enums_query)).fetchall()

        # 조회된 enum 목록을 기반으로 모든 enum 삭제
        for enum_schema, enum_name in result:
            try:
                # ENUM 타입 삭제
                drop_query = f"DROP TYPE IF EXISTS {enum_schema}.{enum_name} CASCADE;"
                connection.execute(text(drop_query))
                print(f"ENUM type '{enum_schema}.{enum_name}' has been dropped.")
            except Exception as e:
                print(f"Error dropping ENUM '{enum_schema}.{enum_name}': {e}")

# 기존 테이블을 지우는 함수
def drop_all_tables():
    with engine.connect() as connection:
        # 모든 테이블 삭제
        try:
            drop_schema_query = "DROP SCHEMA public CASCADE;"
            connection.execute(text(drop_schema_query))  # SQL문을 text로 감싸서 실행
            print("All tables have been dropped.")
        except Exception as e:
            print(f"Error dropping schema: {e}")

# 데이터베이스를 초기화하고 테이블과 enum을 새로 생성하는 함수
def reset_database():
    drop_all_enums()  # 모든 ENUM 삭제
    drop_all_tables()  # 모든 테이블 삭제
    
    # 새롭게 정의된 테이블 및 enum을 생성
    from sqlmodel import SQLModel
    from app.models import Robot, Inventory, Chat  # 예시로 필요한 모델을 import
    SQLModel.metadata.create_all(bind=engine)  # 새로운 모델을 통해 테이블 생성
    print("Database has been reset and tables created with new models.")

# 실행
reset_database()
