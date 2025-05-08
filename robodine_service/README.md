# 🚀 RoboDine 서비스

![](https://img.shields.io/badge/Python-3.8+-blue) ![](https://img.shields.io/badge/FastAPI-0.68.0-green) ![](https://img.shields.io/badge/React-17.0.2-blue) ![](https://img.shields.io/badge/SQLite-3.36-orange) ![](https://img.shields.io/badge/WebSocket-4.0-purple)

## 📌 개요

RoboDine 서비스는 로봇 자동화 레스토랑의 중추적인 중앙 시스템으로, 백엔드 API 서버와 프론트엔드 애플리케이션을 통합한 완전한 서비스 솔루션입니다. 이 시스템은 로봇 제어, 주문 관리, 재고 추적, 사용자 인터페이스를 아우르는 종합적인 플랫폼을 제공합니다.

## 🏛️ 구성 요소

RoboDine 서비스는 두 가지 주요 구성 요소로 이루어져 있습니다:

### 1. [백엔드 서비스](backend)

FastAPI 기반의 강력한 백엔드 시스템으로, 다음과 같은 핵심 기능을 제공합니다:
- REST API 서버
- WebSocket 실시간 통신
- 로봇 제어 인터페이스 (TCP/UDP)
- 데이터 관리 및 비즈니스 로직
- 실시간 비디오 스트리밍 (RTSP)

[백엔드 서비스 문서 바로가기](backend/README.md)

### 2. [프론트엔드 시스템](frontend)

React.js 기반의 사용자 인터페이스 시스템으로, 다음과 같은 애플리케이션을 포함합니다:
- [운영자 대시보드](frontend/operator) - 매장 모니터링 및 관리 인터페이스
- [키오스크 시스템](frontend/kiosk) - 고객 주문 인터페이스

[프론트엔드 시스템 문서 바로가기](frontend/README.md)

## 📊 데이터 흐름

RoboDine 서비스 내에서의 데이터 흐름은 다음과 같습니다:

1. **고객 주문 프로세스**:
   - 키오스크 UI → REST API → 백엔드 서버 → 데이터베이스
   - 백엔드 서버 → WebSocket → 운영자 대시보드 (실시간 알림)
   - 백엔드 서버 → TCP → Cook Planner (조리 지시)

2. **로봇 상태 모니터링**:
   - 로봇 → TCP → 백엔드 서버 → 데이터베이스
   - 백엔드 서버 → WebSocket → 운영자 대시보드 (실시간 상태)
   - 로봇 카메라 → RTSP → 백엔드 RTSP 서버 → 운영자 대시보드

3. **재고 관리**:
   - 운영자 대시보드 → REST API → 백엔드 서버 → 데이터베이스
   - 자동 재고 감소: 주문 → 백엔드 서버 → 데이터베이스

   

## 📚 관련 문서

RoboDine 서비스에 대한 자세한 이해를 위해 다음 문서를 참조하세요:

- [전체 시스템 아키텍처](../docs/architecture.md)
- [FastAPI 백엔드 아키텍처](../docs/fastapi_architecture.md)
- [WebSocket 통신 아키텍처](../docs/websocket_architecture.md)
- [네트워크 통신 방식](../docs/network_communication.md)
- [프로젝트 구조](../docs/project_structure.md)