# 🛠️ RoboDine 시스템 아키텍처

---

## 📐 아키텍처 구성도

```plaintext
[고객 입장] → Alba Planner (좌석 안내) → RoboDine Service (좌석/웨이팅 관리)
   │
[Kiosk 주문] → RoboDine Service (주문 관리)
   │
Cook Planner (조리 지시) ← CookGPT (재료 분류 등)
   │
[조리 완료] → Alba Planner (서빙) → [고객 식사 후 퇴장 관리]

※ 비상 상황 발생 시 RoboDine Core가 Alba Planner를 통해 로봇 즉각 대피
```

---

## 🌳 상세 프로젝트 파일 트리

```plaintext
robodine_project/
├── robodine_service/ (중앙 서버 - 전체 시스템 관제 및 데이터 관리)
│   ├── backend/ (FastAPI REST API 서버)
│   │   ├── run.py (FastAPI 서버 실행)
│   │   ├── requirements.txt (파이썬 종속성 관리)
│   │   ├── app/
│   │   │   ├── models/ (SQLModel ORM 모델)
│   │   │   │   ├── __init__.py (초기화)
│   │   │   │   ├── robot.py (로봇 모델)
│   │   │   │   ├── customer.py (고객 모델)
│   │   │   │   ├── order.py (주문 모델)
│   │   │   │   ├── inventory.py (재고 모델)
│   │   │   │   ├── event.py (이벤트 모델)
│   │   │   │   ├── table.py (테이블 모델)
│   │   │   │   ├── waiting_list.py (대기 리스트 모델)
│   │   │   │   ├── cleaning_task.py (청소 작업 모델)
│   │   │   │   ├── emergency.py (비상 상황 모델)
│   │   │   │   ├── video_stream.py (영상 스트리밍 모델)
│   │   │   │   ├── robot_command.py (로봇 명령 모델)
│   │   │   │   └── admin_settings.py (관리자 설정 모델)
│   │   │   ├── services/ (TCP, UDP, WebSocket 서비스)
│   │   │   │   ├── ros_service.py (ROS2 데이터 수신 및 저장)
│   │   │   │   ├── inventory_service.py (실시간 재고 관리)
│   │   │   │   ├── emergency_service.py (비상 상황 관리)
│   │   │   │   ├── cleaning_service.py (자동 청소 관리)
│   │   │   │   ├── auto_ordering.py (재료 자동 발주)
│   │   │   │   ├── control_service.py (관제 및 로봇 추적)
│   │   │   ├── routes/ (FastAPI 라우트)
│   │   │   │   ├── robots.py
│   │   │   │   ├── websockets.py
│   │   │   │   ├── inventory.py
│   │   │   │   ├── orders.py
│   │   │   │   └── control.py
│   │   │   ├── core/ (공통 모듈 및 DB 연결)
│   │   │   │   ├── config.py (환경설정)
│   │   │   │   ├── db_config.py (환경설정 관리)
│   │   │   │   ├── database.py (DB 연결 관리)
│   │   │   │   └── dependencies.py (FastAPI 종속성 주입)
│   │   ├── db/ (DB 설정)
│   │   │   ├── create_db.py (DB 생성 스크립트)
│   │   └── tests/ (pytest 테스트)
│   │       ├── test_routes/ (라우트 테스트)
│   │       ├── test_services/ (서비스 테스트)
│   │       └── test_models/ (모델 테스트)
│   └── frontend/ (React 웹 인터페이스)
│       ├── operator/ (관리자 관제용)
│       │   ├── components/
│       │   │   ├── Dashboard.jsx (대시보드)
│       │   │   ├── ControlPanel.jsx (관제 패널)
│       │   │   ├── EmergencyView.jsx (비상상황 모니터링)
│       │   │   ├── CleaningView.jsx (청소 현황)
│       │   │   └── InventoryDashboard.jsx (재고 관리)
│       │   ├── pages/
│       │   │   └── OperatorDashboardPage.jsx
│       │   └── tests/ (Jest 테스트)
│       └── kiosk/ (키오스크 주문 인터페이스)
│           ├── components/
│           │   └── KioskInterface.jsx (주문 및 호출)
│           ├── pages/
│           │   └── KioskMain.jsx
│           └── tests/ (Jest 테스트)
│
├── alba_planner/ (핑키 로봇 관리 - 서빙 및 고객 안내)
│   └── alba_manager/
│       ├── ros2_interface.py (Pinky ROS2 인터페이스)
│       ├── tcp_client.py (Core 서버 통신)
│       ├── main.py (실행 진입점)
│       └── tests/ (pytest 테스트)
│
├── cook_planner/ (myCobot 로봇 관리 - 음식 조리 관리)
│   └── cook_manager/
│       ├── ros2_interface.py (myCobot ROS2 인터페이스)
│       ├── udp_interface.py (CookGPT 통신)
│       ├── tcp_client.py (Core 서버 통신)
│       ├── cooking_tasks.py (조리 작업 관리)
│       ├── main.py (실행 진입점)
│       └── tests/ (pytest 테스트)
│
├── cook_gpt/ (AI 기반 조리 지원 및 비전 처리)
│   └── cookgpt_service/
│       ├── inference.py (AI 조리 추론)
│       ├── udp_server.py (Cook Planner UDP 통신)
│       ├── object_detection.py (음식 인식)
│       ├── pose_estimation.py (음식 위치 및 자세 추정)
│       ├── visual_servoing.py (로봇 위치 보정)
│       ├── main.py (실행 진입점)
│       └── tests/ (pytest 테스트)
│
└── docs/ (프로젝트 문서)
    ├── architecture.md (아키텍처 문서)
    ├── system_requirements.md (시스템 요구사항)
    ├── project_structure.md (프로젝트 구조 및 협업 가이드)
    └── github_guide.md (GitHub 협업 가이드)
```

---

## 📡 시스템 통신 프로토콜 및 포트 정의

| 연결 주체                        | 프로토콜   | 포트  | 역할 및 기능                 |
|----------------------------------|----------|------|--------------------------|
| RoboDine Service ↔ Alba Planner     | TCP      | 8001 | 고객 안내 및 서빙 지시       |
| RoboDine Service ↔ Cook Planner     | TCP      | 8002 | 음식 주문 전달 및 조리 지시  |
| CookGPT ↔ Cook Planner           | UDP      | 8003 | AI 기반 조리 정보 전달      |
| RoboDine Service ↔ Frontend(Web)    | WebSocket| 3000 | 실시간 매장 상태 업데이트   |

---

## 📌 각 모듈 설명

- **RoboDine Service**: 중앙 서버로, 데이터 관리, 관제 및 시스템 통합 수행
- **Alba Planner**: 핑키 로봇 제어 및 고객 응대, 좌석 안내, 서빙 관리
- **Cook Planner**: 로봇팔(myCobot280)을 이용한 음식 조리 작업 관리
- **CookGPT**: AI 음식 조리 지원 및 컴퓨터 비전 기능 제공
