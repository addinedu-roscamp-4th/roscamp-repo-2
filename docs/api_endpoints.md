# 🌐 RoboDine API 엔드포인트 문서

## 📌 개요

이 문서는 RoboDine 서비스의 API 엔드포인트에 대한 상세 정보를 제공합니다. 이 API는 클라이언트(운영자 대시보드 및 키오스크), 로봇 제어 시스템, 외부 시스템과의 통신을 위한 인터페이스를 제공합니다.

## 🔐 인증

대부분의 엔드포인트는 인증이 필요합니다. 인증은 JWT(JSON Web Token) 기반으로 구현되어 있습니다.

### 인증 토큰 획득

```
URL: /api/auth/login
Method: POST
Content-Type: application/json
```

#### 요청 예시

```json
{
  "username": "admin",
  "password": "securepassword"
}
```

#### 응답 예시

```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer",
  "expires_in": 3600
}
```

### 토큰 갱신

```
URL: /api/auth/refresh
Method: POST
Authorization: Bearer {refresh_token}
```

## 🤖 로봇 관리 API

### 모든 로봇 조회

```
URL: /api/robots
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 1,
    "robot_id": "alba-001",
    "type": "SERVING",
    "status": "IDLE",
    "location": {"x": 10.5, "y": 20.3},
    "battery_level": 87,
    "last_updated": "2023-08-15T14:30:22"
  },
  {
    "id": 2,
    "robot_id": "cook-001",
    "type": "COOKING",
    "status": "BUSY",
    "location": {"x": 2.5, "y": 5.1},
    "battery_level": 92,
    "last_updated": "2023-08-15T14:30:10"
  }
]
```

### 특정 로봇 상세 정보

```
URL: /api/robots/{robot_id}
Method: GET
Authorization: Bearer {access_token}
```

### 로봇 명령 전송

```
URL: /api/robots/{robot_id}/command
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "command": "MOVE",
  "parameters": {
    "destination": {"x": 15.2, "y": 8.7},
    "speed": 0.5
  }
}
```

## 📋 주문 관리 API

### 주문 목록 조회

```
URL: /api/orders
Method: GET
Authorization: Bearer {access_token}
Query Parameters:
  - status: (optional) 주문 상태 필터 (PENDING, COOKING, SERVING, COMPLETED, CANCELLED)
  - from_date: (optional) 시작 날짜 (YYYY-MM-DD)
  - to_date: (optional) 종료 날짜 (YYYY-MM-DD)
  - limit: (optional) 반환할 최대 주문 수
  - offset: (optional) 페이지네이션 오프셋
```

### 주문 생성

```
URL: /api/orders
Method: POST
Authorization: Bearer {access_token} (키오스크의 경우 인증 면제 가능)
Content-Type: application/json
```

#### 요청 예시

```json
{
  "table_id": 5,
  "customer_count": 2,
  "items": [
    {
      "menu_id": 101,
      "quantity": 1,
      "options": [
        {"option_id": 15, "value": "extra_cheese"}
      ],
      "special_instructions": "소스 적게 넣어주세요"
    },
    {
      "menu_id": 203,
      "quantity": 2,
      "options": []
    }
  ],
  "payment_method": "CARD",
  "payment_status": "PAID"
}
```

### 주문 상태 업데이트

```
URL: /api/orders/{order_id}/status
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "status": "COOKING",
  "estimated_completion_time": "2023-08-15T15:00:00"
}
```

## 🍽️ 메뉴 API

### 메뉴 카테고리 조회

```
URL: /api/menu/categories
Method: GET
```

### 메뉴 항목 조회

```
URL: /api/menu/items
Method: GET
Query Parameters:
  - category_id: (optional) 카테고리 ID
  - is_available: (optional) 재고 여부 필터
```

### 메뉴 항목 상세 정보

```
URL: /api/menu/items/{item_id}
Method: GET
```

## 🏢 테이블 관리 API

### 테이블 목록 조회

```
URL: /api/tables
Method: GET
Authorization: Bearer {access_token}
```

### 특정 테이블 상태 조회

```
URL: /api/tables/{table_id}
Method: GET
Authorization: Bearer {access_token}
```

### 테이블 상태 업데이트

```
URL: /api/tables/{table_id}/status
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "status": "OCCUPIED",
  "customer_count": 3
}
```

## 📦 재고 관리 API

### 재고 목록 조회

```
URL: /api/inventory
Method: GET
Authorization: Bearer {access_token}
```

### 재고 업데이트

```
URL: /api/inventory/{item_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "quantity": 25,
  "threshold": 10,
  "auto_order": true
}
```

## 📊 시스템 이벤트 API

### 이벤트 목록 조회

```
URL: /api/events
Method: GET
Authorization: Bearer {access_token}
Query Parameters:
  - type: (optional) 이벤트 타입 (SYSTEM, ROBOT, ORDER, INVENTORY)
  - severity: (optional) 심각도 (INFO, WARNING, ERROR, CRITICAL)
  - limit: (optional) 반환할 최대 이벤트 수
  - offset: (optional) 페이지네이션 오프셋
```

### 특정 이벤트 상세 정보

```
URL: /api/events/{event_id}
Method: GET
Authorization: Bearer {access_token}
```

## 📹 비디오 스트림 API

### 스트림 목록 조회

```
URL: /api/video-streams
Method: GET
Authorization: Bearer {access_token}
```

### 특정 스트림 정보

```
URL: /api/video-streams/{stream_id}
Method: GET
Authorization: Bearer {access_token}
```

### 녹화 시작

```
URL: /api/video-streams/{stream_id}/record
Method: POST
Authorization: Bearer {access_token}
```

## 🔄 WebSocket API

WebSocket 연결은 다음과 같은 토픽별 실시간 업데이트를 위해 사용됩니다:

### 연결 엔드포인트

```
URL: /ws/{topic}
Authorization: Bearer {access_token} as query parameter
```

유효한 토픽:
- `robots`: 로봇 상태 업데이트
- `orders`: 주문 상태 변경
- `tables`: 테이블 상태 업데이트
- `events`: 이벤트 발생 알림
- `inventory`: 재고 변경 알림
- `systemlogs`: 시스템 로그 스트리밍

### 메시지 형식

모든 WebSocket 메시지는 JSON 형식을 따르며 다음과 같은 구조를 가집니다:

```json
{
  "type": "update",
  "topic": "robots",
  "data": {
    // 토픽별 데이터 구조
  }
}
```

메시지 타입:
- `update`: 데이터 업데이트
- `error`: 오류 메시지
- `ping`: 연결 유지 확인 요청
- `pong`: 연결 유지 응답
- `shutdown`: 서버 종료 알림

## 📝 오류 코드 및 응답

API 오류 응답은 다음 형식을 따릅니다:

```json
{
  "detail": "오류 메시지",
  "code": "ERROR_CODE",
  "status_code": 400
}
```

| 오류 코드 | HTTP 상태 코드 | 설명 |
|---------|--------------|-----|
| `AUTHENTICATION_REQUIRED` | 401 | 인증이 필요함 |
| `INVALID_CREDENTIALS` | 401 | 잘못된 인증 정보 |
| `TOKEN_EXPIRED` | 401 | 토큰 만료 |
| `PERMISSION_DENIED` | 403 | 권한 없음 |
| `RESOURCE_NOT_FOUND` | 404 | 리소스를 찾을 수 없음 |
| `VALIDATION_ERROR` | 422 | 요청 데이터 유효성 검증 실패 |
| `ROBOT_UNAVAILABLE` | 409 | 로봇이 현재 사용 불가능 |
| `INSUFFICIENT_INVENTORY` | 409 | 재고 부족 |
| `SYSTEM_ERROR` | 500 | 서버 내부 오류 | 