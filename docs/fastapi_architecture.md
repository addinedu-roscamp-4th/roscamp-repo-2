# FastAPI 기반 로보다인 백엔드 아키텍처

## 📌 개요

로보다인 서비스의 백엔드는 FastAPI 프레임워크를 기반으로 구축되어 있으며, 실시간 데이터 처리와 다양한 통신 프로토콜을 지원하는 고성능 서버 시스템입니다. 이 문서는 백엔드 아키텍처의 핵심 구성 요소와 데이터 흐름을 설명합니다.

## 🏗️ 아키텍처 구성

### 핵심 구성 요소

```
backend/
├── app/
│   ├── core/         # 핵심 기능 및 설정
│   ├── models/       # 데이터 모델 (SQLModel)
│   ├── routes/       # API 엔드포인트
│   └── services/     # 비즈니스 로직 서비스
├── run.py            # 메인 애플리케이션 진입점
├── UDP_receiver.py   # UDP 통신 처리
└── requirements.txt  # 의존성 패키지
```

## 🔄 API 구조 및 라우팅

FastAPI는 라우터를 통해 모듈화된 API 구조를 제공하며, 각 기능 영역별로 분리된 라우터를 사용합니다:

### 주요 라우터 목록

| 라우터 | 경로 | 설명 |
|--------|------|------|
| `auth.router` | `/api/auth` | 인증 및 토큰 관리 |
| `users.router` | `/api/users` | 사용자 관리 |
| `robot.router` | `/api/robots` | 로봇 관리 API |
| `albabot.router` | `/api/albabot` | 알바봇 관련 API |
| `cookbot.router` | `/api/cookbot` | 쿡봇 관련 API |
| `tables.router` | `/api/tables` | 테이블 관리 API |
| `orders.router` | `/api/orders` | 주문 관리 API |
| `menu.router` | `/api/menu` | 메뉴 관리 API |
| `inventories.router` | `/api/inventory` | 재고 관리 API |
| `events.router` | `/api/events` | 이벤트 관리 API |
| `video_streams.router` | `/api/video-streams` | 비디오 스트림 API |
| `websocket_router` | `/ws/{topic}` | 웹소켓 연결 관리 |

## 💡 핵심 기능

### 1. 비동기 API 처리

FastAPI는 Starlette와 Pydantic을 기반으로 비동기 API 요청을 효율적으로 처리합니다:

```python
@app.get("/health")
async def health_check():
    return {"status": "ok"}
```

### 2. 종속성 주입 시스템

데이터베이스 세션 및 기타 리소스에 대한 종속성 주입 패턴:

```python
from app.core.database import get_session

@router.get("/items")
async def get_items(session: Session = Depends(get_session)):
    items = session.exec(select(Item)).all()
    return items
```

### 3. 자동 API 문서화

FastAPI는 OpenAPI 및 Swagger UI를 통해 자동으로 API 문서를 생성합니다:
- `/docs` : Swagger UI 기반 대화형 문서
- `/redoc` : ReDoc 기반 문서

### 4. 정적 파일 서비스

비디오 및 기타 정적 파일을 서비스하기 위한 구성:

```python
VIDEOS_DIR = "/home/addinedu/dev_ws/roscamp-repo-2/robodine_service/backend/videos"
app.mount("/videos", StaticFiles(directory=VIDEOS_DIR), name="videos")
```

## 🔄 데이터 흐름

### 1. HTTP 요청 처리

1. 클라이언트가 API 엔드포인트에 요청
2. FastAPI 미들웨어가 요청 처리 (CORS, GZip 등)
3. 경로 연산 함수가 요청 처리 및 비즈니스 로직 실행
4. 데이터베이스 액세스 및 데이터 처리
5. JSON 응답 반환

### 2. 웹소켓 통신

1. 클라이언트가 `/ws/{topic}` 엔드포인트에 웹소켓 연결 요청
2. `ConnectionManager`가 연결 관리 및 토픽별 구독 처리
3. 데이터베이스 변경 사항이 발생하면 해당 토픽 구독자에게 실시간 업데이트
4. 클라이언트 연결이 종료되면 관리자가 연결 해제 처리

### 3. TCP/UDP 통신

1. 외부 시스템(로봇)이 TCP/UDP 포트로 데이터 전송
2. 전용 핸들러가 데이터 파싱 및 처리
3. 데이터베이스에 저장 및 웹소켓을 통해 클라이언트에 통보

## 🔧 확장성 및 성능 최적화

### 1. 비동기 처리

FastAPI는 Python의 `asyncio`를 활용하여 I/O 바운드 작업의 동시 처리:

```python
async def fallback_polling():
    """데이터베이스 폴링 백업 메커니즘 (이벤트 놓침 대비)"""
    while True:
        # 비동기 작업 수행
        await asyncio.sleep(POLLING_INTERVAL)
```

### 2. 미들웨어 최적화

CORS 및 압축 미들웨어를 통한 성능 최적화:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.add_middleware(
    GZipMiddleware,
    minimum_size=1000,
)
```

### 3. 백그라운드 작업

`BackgroundTasks`를 사용한 비동기 백그라운드 작업 처리:

```python
@router.post("/process")
async def process_data(background_tasks: BackgroundTasks):
    background_tasks.add_task(long_running_operation)
    return {"message": "Processing started"}
```

## 📋 모델 및 데이터베이스

SQLModel(SQLAlchemy + Pydantic)을 사용한 ORM 구현:

```python
from sqlmodel import SQLModel, Field

class Robot(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    robot_id: str
    type: EntityType
    mac_address: str
    ip_address: str
    timestamp: datetime = Field(default_factory=datetime.now)
```

## 🚀 애플리케이션 생명주기

```python
@asynccontextmanager
async def lifespan(app: FastAPI):
    # 시작 시 초기화 작업
    SQLModel.metadata.create_all(bind=engine)
    
    # 비디오 디렉토리 생성
    os.makedirs(VIDEOS_DIR, exist_ok=True)
    
    # TCP 서버 시작
    tcp_thread = threading.Thread(target=start_tcp_server, args=("0.0.0.0", 8001), daemon=True)
    tcp_thread.start()
    
    # 백업 폴링 시작
    fallback_task = asyncio.create_task(fallback_polling())
    
    yield  # 애플리케이션 실행
    
    # 종료 시 정리 작업
    fallback_task.cancel()
```

## 📊 모니터링 및 로깅

```python
logger = logging.getLogger("robodine.run")
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
```

## 🛠️ 확장 가능성

1. 추가 로봇 타입 및 프로토콜 통합
2. 실시간 분석 및 모니터링 강화
3. 마이크로서비스 아키텍처로의 확장

## 📝 결론

FastAPI 기반 로보다인 백엔드는 고성능, 확장성 및 개발자 친화적인 구조를 갖춘 현대적인 API 서버입니다. 비동기 처리, 실시간 통신, 다양한 프로토콜 지원을 통해 복잡한 로봇 서비스 관리를 효율적으로 처리합니다. 