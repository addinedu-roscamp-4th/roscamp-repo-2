from sqlmodel import create_engine, SQLModel, Session
from sqlalchemy.orm import declarative_base
from sqlalchemy.orm import sessionmaker

DATABASE_URL = "sqlite:///./robodine.db"

# 데이터베이스 연결 엔진 생성
engine = create_engine(DATABASE_URL, echo=True, connect_args={"check_same_thread": False})

# 세션 생성기 설정
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# 기본 테이블 클래스 설정
Base = declarative_base()

def get_db():
    """Dependency to get a database session."""
    db = Session(engine)
    try:
        yield db
    finally:
        db.close()

# 의존성 주입을 위한 세션 팩토리
def get_session():
    with Session(engine) as session:
        return session
