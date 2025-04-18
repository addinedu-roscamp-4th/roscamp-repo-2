from sqlmodel import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker

DATABASE_URL = "postgresql+psycopg2://robodine_user:robodine_pass@localhost:5432/robodine_db"

# 데이터베이스 연결 엔진 생성
engine = create_engine(DATABASE_URL, echo=True)

# 세션 생성기 설정
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# 기본 테이블 클래스 설정
Base = declarative_base()
