# 🤖 RoboDine - AI 기반 로봇 자동화 레스토랑 플랫폼

![RoboDine 로고](/docs/images/robodine_logo.png)

> *로봇과 AI가 만드는 스마트 레스토랑의 미래*

![](https://img.shields.io/badge/Python-3.8+-blue) ![](https://img.shields.io/badge/FastAPI-0.68.0-green) ![](https://img.shields.io/badge/React-17.0.2-blue) ![](https://img.shields.io/badge/ROS2-Jazzy-orange) ![](https://img.shields.io/badge/PyTorch-1.10-red) ![](https://img.shields.io/badge/TensorRT-8.2-blueviolet) ![](https://img.shields.io/badge/WebSocket-4.0-purple)

## 📌 프로젝트 개요

RoboDine은 로봇 기술과 인공지능을 결합한 완전 자동화된 레스토랑 서비스 플랫폼입니다. 고객 응대부터 주문 처리, 음식 조리, 서빙까지 로봇을 통해 자동화하여 효율적이고 일관된 서비스를 제공합니다.

### 🎯 주요 목표

- 로봇을 활용한 레스토랑 자동화를 통한 인건비 절감 및 서비스 효율성 향상
- 인공지능을 활용한 조리 과정 최적화 및 품질 일관성 유지
- 실시간 모니터링 및 원격 관리를 통한 매장 운영 효율성 증대
- 고객 경험 향상을 위한 직관적인 인터페이스 및 서비스 제공

## 💻 기술 스택

| 영역 | 기술 |
|-----|-----|
| **백엔드** | Python, FastAPI, SQLModel, WebSocket, RTSP |
| **프론트엔드** | React.js, Tailwind CSS, Chart.js, Socket.io-client |
| **로봇 제어** | ROS2 Jazzy, PyRobots, myCobot SDK |
| **AI & 비전** | PyTorch, OpenCV, TensorRT, YOLOv8 |
| **통신** | WebSocket, TCP, UDP, REST API, RTSP |
| **데이터베이스** | SQLite (개발), PostgreSQL (프로덕션) |
| **개발 도구** | Docker, Git, GitHub Actions, Pytest, Jest |

## 🏗️ 시스템 구성

RoboDine 시스템은 다음 핵심 모듈로 구성됩니다:

```mermaid
graph TD
    A[RoboDine 플랫폼] --> B[RoboDine Service]
    A --> C[Alba Planner]
    A --> D[Cook Planner]
    A --> E[CookGPT]
    
    B --> B1[백엔드 서비스]
    B --> B2[프론트엔드 시스템]
    B2 --> B2a[운영자 대시보드]
    B2 --> B2b[키오스크 시스템]
    
    C --> C1[고객 안내 서비스]
    C --> C2[음식 서빙 서비스]
    C --> C3[테이블 관리]
    
    D --> D1[조리 작업 관리]
    D --> D2[조리 과정 최적화]
    D --> D3[안전 제어]
    
    E --> E1[컴퓨터 비전 처리]
    E --> E2[위치 추정]
    E --> E3[로봇 제어 보조]
    
    class A main
    class B,C,D,E modules
    
    classDef main fill:#f9d5e5,stroke:#333,stroke-width:2px
    classDef modules fill:#eeeeee,stroke:#333,stroke-width:1px
```

### 1. [RoboDine Service](/robodine_service)
- **[백엔드 서비스](/robodine_service/backend)**: FastAPI 기반의 고성능 API 서버
- **[프론트엔드 시스템](/robodine_service/frontend)**: React 기반의 사용자 인터페이스
  - **[운영자 대시보드](/robodine_service/frontend/operator)**: 매장 운영 모니터링 및 관리
  - **[키오스크 시스템](/robodine_service/frontend/kiosk)**: 고객 주문 인터페이스

### 2. [Alba Planner](/alba_planner)
서빙 로봇(Pinky)을 제어하는 시스템으로, 음식 서빙, 테이블 관리 기능을 제공합니다.

### 3. [Cook Planner](/cook_planner)
조리 로봇(JetCobot)을 제어하는 시스템으로, 정밀한 조리 작업을 관리하고 최적화합니다.

### 4. [CookGPT](/cook_gpt)
AI 기반 조리 지원 시스템으로, 컴퓨터 비전과 딥러닝을 활용해 로봇의 조리 과정을 개선합니다.

## 📊 시스템 아키텍처

```mermaid
graph TB
    A[고객] --> B[키오스크 시스템]
    A --> C[서빙 로봇<br>고객 응대 & 안내]
    
    subgraph "중앙 서버 시스템"
        D[RoboDine 백엔드<br>FastAPI] <--> E[데이터베이스]
        D <--> F[WebSocket 서비스]
        D <--> G[비디오 스트리밍]
    end
    
    subgraph "로봇 제어 시스템"
        H[Alba Planner<br>서빙 로봇 제어] <--> I[서빙 로봇(Pinky)]
        J[Cook Planner<br>조리 로봇 제어] <--> K[조리 로봇(myCobot)]
    end
    
    subgraph "AI 처리 시스템"
        L[CookGPT<br>조리 지원 AI] <--> M[컴퓨터 비전<br>객체 인식]
    end
    
    B --> D
    C <--> D
    D <--> H
    D <--> J
    D <--> N[운영자 대시보드]
    J <--> L
```

자세한 내용은 다음 기술 문서를 참조하세요:
- [전체 시스템 아키텍처](docs/architecture.md)
- [프로젝트 구조 개요](docs/project_structure.md)
- [FastAPI 아키텍처](docs/fastapi_architecture.md)
- [WebSocket 아키텍처](docs/websocket_architecture.md)
- [네트워크 통신](docs/network_communication.md)
- [API 엔드포인트](docs/api_endpoints.md)
- [데이터베이스 스키마](docs/database_schema.md)
- [오류 코드 참조](docs/error_codes.md)

## ✨ 주요 기능

### 🤖 로봇 서빙 시스템

```mermaid
graph LR
    A[주문 접수] --> B[경로 계산]
    B --> C[음식 픽업]
    C --> D[테이블 이동]
    D --> E[서빙 완료]
    E --> F[충전소 복귀]
    
    style A fill:#f9f9f9,stroke:#333
    style B fill:#f9f9f9,stroke:#333
    style C fill:#f9f9f9,stroke:#333
    style D fill:#f9f9f9,stroke:#333
    style E fill:#f9f9f9,stroke:#333
    style F fill:#f9f9f9,stroke:#333
```

- **주문 서빙**: 주문된 음식을 정확한 테이블로 서빙
- **테이블 관리**: 테이블 상태 모니터링 및 고객 그룹 관리
- **자율 주행**: 장애물 회피 및 최적 경로 계산
- **배터리 관리**: 자동 충전 스테이션 복귀

### 👨‍🍳 로봇 조리 시스템

- **정밀 조리 작업**: 6축 로봇 팔을 활용한 정확한 조리 작업
- **레시피 해석**: 표준화된 레시피 단계별 실행
- **조리 상태 모니터링**: 컴퓨터 비전 기반 위치 상태 확인
- **다중 주문 처리**: 여러 주문의 효율적 병렬 처리
- **안전 제어**: 안전 프로토콜에 따른 로봇 작동 관리

### 💼 매장 관리 시스템

- **실시간 모니터링**: 로봇, 주문, 테이블 등 매장 상태 실시간 모니터링
- **재고 관리**: 자동화된 재고 추적 및 발주 시스템
- **이벤트 처리**: 비상 상황 및 이벤트 실시간 처리
- **데이터 분석**: 운영 데이터 시각화 및 인사이트 제공
- **비디오 관제**: 로봇 카메라 및 매장 CCTV 통합 모니터링

### 📱 고객 인터페이스

- **키오스크 주문**: 직관적인 터치스크린 기반 메뉴 주문 시스템
- **실시간 상태 추적**: 주문 상태 실시간 업데이트
- **다국어 지원**: 한국어, 영어, 중국어, 일본어 지원
- **결제 시스템**: 다양한 결제 방식 지원 (신용카드, 모바일 결제 등)

## 🔄 데이터 흐름

```mermaid
sequenceDiagram
    participant Customer as 고객
    participant Kiosk as 키오스크
    participant Backend as 백엔드
    participant Cook as 조리 로봇
    participant Alba as 서빙 로봇
    participant Dashboard as 대시보드
    
    Customer->>Kiosk: 주문 입력
    Kiosk->>Backend: 주문 전송
    Backend->>Dashboard: 주문 알림
    Backend->>Cook: 조리 명령
    Cook->>Backend: 조리 완료
    Backend->>Alba: 서빙 명령
    Alba->>Customer: 음식 서빙
    Alba->>Backend: 서빙 완료
    Backend->>Dashboard: 상태 업데이트
    Backend->>Kiosk: 주문 완료 알림
```

## 📡 네트워크 통신

RoboDine은 다양한 통신 프로토콜을 활용하여 로봇, 서버, 클라이언트 간의 효율적인 데이터 교환을 구현합니다:

| 프로토콜 | 포트 | 용도 | 구현 위치 |
|---------|------|-----|----------|
| HTTP/REST | 8000 | 데이터 CRUD, 인증 | FastAPI 백엔드 |
| WebSocket | 3000 | 실시간 데이터 업데이트 | ConnectionManager |
| TCP | 8001/8002 | 로봇 명령 및 상태 | RoboDineTCPHandler |
| UDP | 8003 | 비디오 스트림, 센서 데이터 | UDP_receiver.py |
| RTSP | 8554 | 실시간 비디오 스트리밍 | RTSP 서버 |

## 🖥️ 사용자 인터페이스

RoboDine은 사용자 역할에 따라 두 종류의 인터페이스를 제공합니다:

### 운영자 대시보드

운영자를 위한 중앙 관제 시스템으로, 다음과 같은 주요 기능을 포함합니다:
- 로봇 상태 모니터링 및 원격 제어
- 주문 및 테이블 관리
- 재고 및 매출 분석
- 시스템 이벤트 및 로그 확인

### 키오스크 시스템

고객이 직접 사용하는 주문 시스템으로, 다음과 같은 특징을 가집니다:
- 직관적인 메뉴 탐색 및 주문 인터페이스
- 주문 상태 실시간 추적
- 다양한 결제 방식 지원
- 다국어 인터페이스

## 🧪 테스트 및 품질 관리

RoboDine은 안정적인 서비스 제공을 위해 철저한 테스트 프로세스를 구축했습니다:

- **단위 테스트**: 개별 함수 및 컴포넌트의 정확성 검증
- **통합 테스트**: 모듈 간 상호작용 및 데이터 흐름 검증

## 👥 개발팀

- 김용규(팀장) - 조리 로봇 제어
- 김인수 - 통신 및 웹 프론트엔드
- 임지혜 - AI 및 비전 시스템
- 장성원 - AI 및 비전 시스템
- 심경용 - 서빙 로봇 제어
- 허은재 - 서빙 로봇 제어




