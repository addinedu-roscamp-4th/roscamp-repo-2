from sqlmodel import SQLModel, create_engine
from sqlalchemy.ext.asyncio import create_async_engine, AsyncEngine
from sqlmodel.ext.asyncio.session import AsyncSession
from sqlalchemy.orm import sessionmaker

# 비동기 드라이버(asyncpg) 사용
ASYNC_DATABASE_URL = "postgresql+asyncpg://robodine_user:robodine_pass@localhost:5432/robodine_db"
async_engine: AsyncEngine = create_async_engine(
    ASYNC_DATABASE_URL,
    echo=True,
)

# 비동기 세션 생성기
AsyncSessionLocal = sessionmaker(
    bind=async_engine,
    class_=AsyncSession,
    expire_on_commit=False,
    autoflush=False,
    autocommit=False
)

# 동기 엔진 (기존 코드와 호환되는 engine 변수 추가)
SYNC_DATABASE_URL = "postgresql://robodine_user:robodine_pass@localhost:5432/robodine_db"
engine = create_engine(
    SYNC_DATABASE_URL,
    echo=True,
)

# 동기 세션 생성기
SessionLocal = sessionmaker(
    bind=engine,
    autoflush=False,
    autocommit=False
)

# 기본 모델 클래스
Base = SQLModel

# 동기 세션 의존성
def get_db():
    """Dependency to get a synchronous database session."""
    with SessionLocal() as session:
        yield session

# 비동기 세션 의존성
async def get_async_db():
    """Dependency to get an asynchronous database session."""
    async with AsyncSessionLocal() as session:
        yield session
