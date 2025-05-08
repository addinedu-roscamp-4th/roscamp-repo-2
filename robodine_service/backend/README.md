# 🚀 RoboDine 백엔드 서비스

![](https://img.shields.io/badge/Python-3.8+-blue) ![](https://img.shields.io/badge/FastAPI-0.68.0-green) ![](https://img.shields.io/badge/SQLModel-0.0.6-orange) ![](https://img.shields.io/badge/Socket.io-5.0.1-purple) ![](https://img.shields.io/badge/RTSP-Server-yellow)

## 📌 개요

RoboDine 백엔드 서비스는 로봇 자동화 레스토랑의 핵심 중앙 통제 시스템으로, 다양한 로봇과 서비스 간의 통신을 조정하고 비즈니스 로직을 처리합니다. FastAPI 기반의 고성능 비동기 서버를 통해 실시간 데이터 처리와 로봇 제어가 가능합니다.

## 🏗 아키텍처

백엔드 서비스는 다음과 같은 핵심 컴포넌트로 구성됩니다:

- **REST API 서버**: FastAPI 기반의 RESTful API 서비스
- **WebSocket 서버**: 실시간 데이터 업데이트를 위한 양방향 통신
- **TCP/UDP 통신 모듈**: 로봇 통신을 위한 소켓 서버
- **RTSP 스트리밍 서버**: 로봇 카메라 영상 스트리밍
- **ORM 및 데이터 레이어**: SQLModel을 활용한 데이터베이스 연동

자세한 아키텍처 설명은 다음 문서를 참조하세요:
- [FastAPI 아키텍처 문서](../../docs/fastapi_architecture.md)
- [WebSocket 아키텍처 문서](../../docs/websocket_architecture.md)
- [네트워크 통신 문서](../../docs/network_communication.md)

## 📁 디렉토리 구조

```
backend/
├── run.py                  # 애플리케이션 엔트리포인트
├── requirements.txt        # 의존성 목록
├── app/                    # 애플리케이션 코드
│   ├── core/               # 핵심 모듈 (설정, DB 연결 등)
│   ├── models/             # SQLModel 데이터 모델
│   ├── services/           # 비즈니스 로직 및 서비스
│   ├── routes/             # API 라우트 정의
│   ├── utils/              # 유틸리티 함수
│   └── middlewares/        # FastAPI 미들웨어
├── db/                     # 데이터베이스 관련 파일
│   └── create_db.py        # DB 초기화 스크립트
└── tests/                  # 테스트 코드
    ├── test_routes/        # API 라우트 테스트
    ├── test_services/      # 서비스 테스트
    └── test_models/        # 모델 테스트
```

## ⚙️ 핵심 기능

### 1. RESTful API

- **사용자 관리**: 인증, 권한 관리
- **로봇 관리**: 로봇 등록, 상태 관리, 명령 전송
- **주문 관리**: 주문 접수, 처리, 상태 추적
- **재고 관리**: 재료 재고 관리, 자동 발주
- **테이블 관리**: 테이블 상태 관리, 예약 처리
- **이벤트 처리**: 비상 상황 감지 및 처리

### 2. 실시간 통신

- **WebSocket**: 클라이언트에 실시간 업데이트 제공
- **TCP 소켓**: 로봇과의 안정적인 양방향 통신
- **UDP 통신**: 로봇 센서 데이터 및 비디오 스트림

### 3. 데이터 관리

- **ORM 모델링**: SQLModel을 통한 타입 안전 데이터 액세스
- **마이그레이션**: Alembic을 통한 데이터베이스 스키마 관리
- **캐싱**: Redis를 통한 빈번한 요청 데이터 캐싱

### 4. 비디오 스트리밍

- **RTSP 서버**: 로봇 카메라에서 실시간 비디오 스트리밍
- **비디오 처리**: OpenCV를 활용한 영상 처리 및 인식





## 🔄 통신 프로토콜

| 포트 | 프로토콜 | 용도 |
|-----|---------|------|
| 8000 | HTTP/REST | REST API 요청 처리 |
| 3000 | WebSocket | 실시간 데이터 업데이트 |
| 8001 | TCP | 서빙 로봇(Alba) 통신 |
| 8002 | TCP | 조리 로봇(Cook) 통신 |
| 8554 | RTSP | 비디오 스트리밍 |



## 🔐 인증 및 보안

- **JWT 인증**: 액세스 토큰 및 리프레시 토큰 기반 인증
- **역할 기반 접근 제어**: 관리자, 직원 등 역할별 권한 관리
- **HTTPS**: TLS/SSL을 통한 API 암호화
- **API 키**: 로봇 및 외부 시스템 인증용 API 키 관리

## 📊 모니터링 및 로깅

- **구조화된 로깅**: JSON 형식의 로그 기록
- **메트릭 수집**: Prometheus 호환 메트릭 엔드포인트
- **트레이싱**: OpenTelemetry 통합 (옵션)
- **알림**: 중요 이벤트 발생 시 Slack/이메일 알림


## 📚 추가 문서

- [API 엔드포인트 문서](../../docs/api_endpoints.md)
- [데이터베이스 스키마](../../docs/database_schema.md)
- [오류 코드 참조](../../docs/error_codes.md)
- [자주 묻는 질문(FAQ)](../../docs/faq.md)
- [인터페이스 명세서](../../docs/interface_specification.md)

## 🚀 성능 최적화 팁

- 인덱스 최적화를 통한 데이터베이스 쿼리 성능 향상
- WebSocket 연결 풀링을 통한 리소스 사용 최적화
- 백그라운드 작업을 위한 Celery 작업 큐 활용
- Redis 캐싱을 통한 반복 요청 응답 시간 단축