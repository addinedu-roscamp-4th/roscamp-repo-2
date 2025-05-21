# 🌐 RoboDine API 엔드포인트 문서

## 📌 개요

이 문서는 RoboDine 서비스의 API 엔드포인트에 대한 상세 정보를 제공합니다. 이 API는 클라이언트(운영자 대시보드 및 키오스크), 로봇 제어 시스템, 외부 시스템과의 통신을 위한 인터페이스를 제공합니다.

## 🔹 Enum 정의

```python
class TaskType(str, Enum):
    GREETINGS = "GREETINGS"
    MAINTENANCE = "MAINTENANCE"
    TAKE_PICTURE = "TAKE_PICTURE"

class EntityType(str, Enum):
    COOKBOT = "COOKBOT"
    ALBABOT = "ALBABOT"
    PINKY = "PINKY"
    GLOBAL = "GLOBAL"
    WORLD = "WORLD"
    INVENTORY = "INVENTORY"
    CHATBOT = "CHATBOT"

class RobotStatus(str, Enum):
    IDLE = "IDLE"
    SETTING = "SETTING"
    COOKING = "COOKING"
    PICKUP = "PICKUP"
    SERVING = "SERVING"
    CLEANING = "CLEANING"
    EMERGENCY = "EMERGENCY"
    MAINTENANCE = "MAINTENANCE"
    SECURITY = "SECURITY"
    BIRTHDAY = "BIRTHDAY"

class CommandStatus(str, Enum):
    PENDING = "PENDING"
    SENT = "SENT"
    ACKED = "ACKED"
    EXECUTED = "EXECUTED"
    FAILED = "FAILED"

class InventoryStatus(str, Enum):
    IN_STOCK = "IN_STOCK"
    LOW_STOCK = "LOW_STOCK"
    OUT_OF_STOCK = "OUT_OF_STOCK"

class TableStatus(str, Enum):
    AVAILABLE = "AVAILABLE"
    OCCUPIED = "OCCUPIED"

class OrderStatus(str, Enum):
    PLACED = "PLACED"
    PREPARING = "PREPARING"
    SERVED = "SERVED"
    CANCELLED = "CANCELLED"

class EventType(str, Enum):
    WELCOME = "WELCOME"
    CALL = "CALL"
    BIRTHDAY = "BIRTHDAY"
    EMERGENCY = "EMERGENCY"
    CLEANING = "CLEANING"

class StreamSourceType(str, Enum):
    PINKY = "PINKY"
    COOKBOT = "COOKBOT"
    GLOBAL_CAM = "GLOBAL_CAM"

class StreamStatus(str, Enum):
    ACTIVE = "ACTIVE"
    INACTIVE = "INACTIVE"
    ERROR = "ERROR"

class UserRole(str, Enum):
    ADMIN = "ADMIN"
    KIOSK = "KIOSK"

class NotificationStatus(str, Enum):
    PENDING = "PENDING"
    SENT = "SENT"
    FAILED = "FAILED"
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

## 🔹 TCP 명세서

### 1. 공통 사항

전송 계층: TCP (장애 허용 재전송 제공)

포트 번호: 8001

인코딩: UTF-8

프레이밍: 각 메시지는 슬래시(/)로 구분된 LENGTH<json> 구조를 사용합니다.

LENGTH는 JSON 페이로드 바이트 수를 4바이트 빅엔디안 정수로 표현

예: \x00\x00\x01\x23{"msg_type":"Albabot",...}

타임아웃: 5초 (미응답 시 재전송 또는 연결 재설정)

### 2. 메시지 유형별 명세

#### 2.1. Alba Manager → Robodine Service

##### 2.1.1. Request (msg_type = "Albabot")

| 필드 이름 | 타입 | 필수 | 설명 |
|----------|------|-----|-----|
| msg_type | string | Y | 고정값: Albabot |
| robot_id | int | Y | 로봇 식별자 |
| status | enum | Y | 로봇 상태 (e.g., IDLE,BUSY) |
| battery_level | int | Y | 배터리 잔량(0~100%) |
| pinky_x, y, z | float | Y | 핑키 위치 (로컬 좌표계) |
| pinky_roll, pitch, yaw | float | Y | 핑키 자세 (롤, 피치, 요) |
| global_x, y, z | float | Y | 글로벌 위치 |
| global_roll, pitch, yaw | float | Y | 글로벌 자세 |
| world_x, y, z | float | Y | 월드 좌표계 위치 |
| world_roll, pitch, yaw | float | Y | 월드 자세 |

예시

```
00 00 01 7c {  
  "msg_type":"Albabot",
  "robot_id":123,
  "status":"BUSY",
  "battery_level":85,
  "pinky_x":0.12,
  ...
}
```

##### 2.1.2. Response

| 필드 이름 | 타입 | 필수 | 설명 |
|----------|------|-----|-----|
| status | string | Y | success 또는 error |
| message | string | Y | 처리 결과 설명 |

예시

```
00 00 00 57 {
  "status":"success",
  "message":"정보가 성공적으로 등록되었습니다."
}
```

#### 2.2. Cook Manager → Robodine Service

##### 2.2.1. Request (msg_type = "Cookbot")

| 필드 이름 | 타입 | 필수 | 설명 |
|----------|------|-----|-----|
| msg_type | string | Y | 고정값: Cookbot |
| robot_id | int | Y | 로봇 식별자 |
| status | enum | Y | 로봇 상태 |
| angle_1..6 | float | Y | 6 관절 각도 (rad) |
| endpoint_x..z | float | Y | 엔드이펙터 위치 |
| endpoint_roll, pitch, yaw | float | Y | 엔드이펙터 자세 |

예시

```
00 00 00 a2 { ... }
```

##### 2.2.2. Response

동일한 구조의 상태/메시지 필드

#### 2.3. Cook GPT → Robodine Service

##### 2.3.1. Ingredient

| 필드 이름 | 타입 | 필수 | 설명 |
|----------|------|-----|-----|
| msg_type | string | Y | Ingredient |
| ingredient_id | int | Y | 재고 식별자 |
| x, y, z | float | Y | 위치 |
| roll, pitch, yaw | float | Y | 자세 |

##### 2.3.2. Face

| 필드 이름 | 타입 | 필수 | 설명 |
|----------|------|-----|-----|
| msg_type | string | Y | Face |
| table_id | int | Y | 테이블 식별자 |
| history | JSON | Y | 얼굴 인식 히스토리 |
| nowdetected | int | Y | 현재 감지 프레임 수 |
| reliability | int | Y | 신뢰도 (0~100) |
| exist | bool | Y | 얼굴 존재 여부 |

#### 2.4. Alba GPT → Robodine Service

##### 2.4.1. Chatbot

| 필드 이름 | 타입 | 필수 | 설명 |
|----------|------|-----|-----|
| msg_type | string | Y | Chatbot |
| msg_id | int | Y | 메시지 식별자 |
| question | string | Y | 사용자 질문 |
| robot_id | int | Y | 로봇 식별자 |
| robot_task | enum | Y | 작업 유형 (PICK,PLACE 등) |
| response | string | Y | GPT 응답 |

### 3. 이벤트 기록 (Event Logging)

각 요청 처리 후, 내부적으로 저장되는 이벤트 포맷:

```json
// 공통 이벤트 구조
{
  "entity_id": <robot_id or ingredient_id>,
  "entity_type": <PINKY|GLOBAL|WORLD|COOKBOT|INVENTORY>,
  "timestamp": "YYYY-MM-DDThh:mm:ssZ",
  // 위치/자세 또는 상태/조인트 필드
  ...
}
```

예시:

```json
{
  "entity_id":123,
  "entity_type":"PINKY",
  "timestamp":"2025-05-14T03:21:45Z",
  "x":0.12,
  "y":-0.45,
  "z":1.23,
  "roll":0.01,
  "pitch":-0.02,
  "yaw":3.14
}
```

### 4. 에러 처리

응답(status = error): 아래 필드 추가

- error_code: string (INVALID_PAYLOAD, TIMEOUT, SERVER_ERROR 등)
- details: string (추가 설명) 

# 🌐 REST API 엔드포인트
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

### 로봇 등록

```
URL: /api/robots/register
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "robot_id": 1,
  "robot_type": "ALBABOT",
  "mac_address": "AA:BB:CC:11:22:33",
  "ip_address": "192.168.0.10"
}
```

#### 응답 예시

```json
{
  "robot_id": 1,
  "status": "success",
  "message": "로봇 정보가 성공적으로 등록되었습니다."
}
```

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
    "mac_address": "AA:BB:CC:11:22:33",
    "ip_address": "192.168.0.10",
    "timestamp": "2023-08-15T14:30:22"
  },
  {
    "id": 2,
    "robot_id": "cook-001",
    "type": "COOKING",
    "mac_address": "DD:EE:FF:44:55:66",
    "ip_address": "192.168.0.11",
    "timestamp": "2023-08-15T14:30:10"
  }
]
```

### 특정 로봇 상세 정보

```
URL: /api/robots/{robot_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "id": 1,
  "robot_id": "alba-001",
  "type": "SERVING",
  "mac_address": "AA:BB:CC:11:22:33",
  "ip_address": "192.168.0.10",
  "timestamp": "2023-08-15T14:30:22"
}
```

### 로봇 명령 전송

```
URL: /api/robots/commands/{robot_id}/command
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "robot_id": 1,
  "command": "MOVE",
  "parameters": {
    "x": 5.0, 
    "y": 3.0, 
    "z": 0.0
  }
}
```

#### 응답 예시

```json
{
  "id": 24,
  "status": "success",
  "message": "명령이 성공적으로 전송되었습니다."
}
```

### 명령 상태 변경

```
URL: /api/robots/commands/{command_id}/status
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "id": 24,
  "status": "EXECUTED"
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "명령의 상태가 성공적으로 변경되었습니다."
}
```

### 명령 목록 조회

```
URL: /api/robots/commands/{robot_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 24,
    "command": "MOVE",
    "parameters": {
      "x": 5.0, 
      "y": 3.0, 
      "z": 0.0
    },
    "status": "EXECUTED",
    "timestamp": "2023-08-15T14:35:22",
    "executed_at": "2023-08-15T14:36:05"
  },
  {
    "id": 23,
    "command": "PICKUP",
    "parameters": {},
    "status": "FAILED",
    "timestamp": "2023-08-15T14:32:10",
    "executed_at": null
  }
]
```

## 🍽️ Albabot 전용 API

### 상태 및 배터리 조회

```
URL: /api/albabot/status/{robot_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "robot_id": 1,
  "status": "SERVING",
  "battery_level": 95,
  "timestamp": "2023-08-15T14:42:07"
}
```

### Albabot 상태 업데이트

```
URL: /api/albabot/status/{robot_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "status": "IDLE",
  "battery_level": 92
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "Albabot 상태가 성공적으로 업데이트되었습니다."
}
```

## 👨‍🍳 Cookbot 전용 API

### 상태 조회

```
URL: /api/cookbot/status/{robot_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "robot_id": 2,
  "status": "COOKING",
  "timestamp": "2023-08-15T14:42:07"
}
```

### Cookbot 상태 업데이트

```
URL: /api/cookbot/status/{robot_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "status": "IDLE"
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "Cookbot 상태가 성공적으로 업데이트되었습니다."
}
```

### 준비해야 하는 메뉴 조회

```
URL: /api/cookbot/todo_order
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "order_id": 501,
    "menu_items": [
      {
        "menu_item_id": 601,
        "name": "스테이크",
        "quantity": 2,
        "status": "PREPARING"
      }
    ],
    "status": "PREPARING",
    "ordered_at": "2023-08-15T14:20:00"
  }
]
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


## 👤 사용자 관리 API

### 로그인

```
URL: /api/auth/login
Method: POST
Content-Type: application/json
```

#### 요청 예시

```json
{
  "username": "user1",
  "password": "pass123"
}
```

#### 응답 예시

```json
{
  "access_token": "eyJ...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

### 로그아웃

```
URL: /api/auth/logout
Method: POST
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "로그아웃 완료."
}
```

### 사용자 목록 조회

```
URL: /api/users
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 1,
    "username": "admin",
    "role": "ADMIN",
    "created_at": "2023-01-01T09:00:00Z",
    "updated_at": "2023-03-10T12:00:00Z"
  },
  {
    "id": 2,
    "username": "kiosk1",
    "role": "KIOSK",
    "created_at": "2023-04-28T15:00:00Z",
    "updated_at": "2023-04-28T15:00:00Z"
  }
]
```

### 사용자 상세 조회

```
URL: /api/users/{user_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "id": 2,
  "username": "kiosk1",
  "role": "KIOSK",
  "created_at": "2023-04-28T15:00:00Z",
  "updated_at": "2023-04-28T15:00:00Z"
}
```

### 사용자 생성

```
URL: /api/users
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "username": "newuser",
  "password": "pass123",
  "role": "KIOSK"
}
```

#### 응답 예시

```json
{
  "id": 3,
  "username": "newuser",
  "role": "KIOSK",
  "created_at": "2023-04-28T16:00:00Z",
  "updated_at": "2023-04-28T16:00:00Z"
}
```

### 사용자 수정

```
URL: /api/users/{user_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "password": "newpass",
  "role": "ADMIN"
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "사용자 정보가 성공적으로 수정되었습니다."
}
```

### 사용자 삭제

```
URL: /api/users/{user_id}
Method: DELETE
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "사용자가 성공적으로 삭제되었습니다."
}
```

## 🔔 알림 API

### 알림 목록 조회

```
URL: /api/users/{user_id}/notifications
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 10,
    "type": "ORDER_STATUS",
    "message": "주문이 준비되었습니다.",
    "created_at": "2023-04-28T14:00:00Z",
    "status": "PENDING"
  },
  {
    "id": 9,
    "type": "SYSTEM",
    "message": "재고가 부족합니다.",
    "created_at": "2023-04-28T13:50:00Z",
    "status": "SENT"
  }
]
```

### 알림 생성

```
URL: /api/users/{user_id}/notifications
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "type": "CALL",
  "message": "도움이 필요합니다."
}
```

#### 응답 예시

```json
{
  "id": 11,
  "status": "success",
  "message": "알림이 생성되었습니다."
}
```

### 알림 상태 변경

```
URL: /api/users/{user_id}/notifications/{notification_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "status": "SENT"
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "알림 상태가 업데이트되었습니다."
}
```

## ⚙️ 관리자 설정 API

### 설정 조회

```
URL: /api/settings
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "operation_start": "08:00",
  "operation_end": "22:00",
  "inventory_threshold": 10,
  "alert_settings": {
    "low_stock": true,
    "new_order": true,
    "emergency": true
  }
}
```

### 설정 업데이트

```
URL: /api/settings
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "operation_start": "09:00",
  "operation_end": "21:00",
  "inventory_threshold": 5,
  "alert_settings": {
    "low_stock": true,
    "new_order": false,
    "emergency": true
  }
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "설정이 업데이트되었습니다."
}
```

## 👥 고객 관리 API

### 고객 그룹 목록 조회

```
URL: /api/customers
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 101,
    "count": 4,
    "timestamp": "2023-04-28T14:10:00Z"
  },
  {
    "id": 102,
    "count": 2,
    "timestamp": "2023-04-28T14:15:00Z"
  }
]
```

### 고객 그룹 생성

```
URL: /api/customers
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "count": 3
}
```

#### 응답 예시

```json
{
  "id": 103,
  "count": 3,
  "timestamp": "2023-04-28T16:10:00Z"
}
```

### 고객 그룹 상세 조회

```
URL: /api/customers/{customer_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "id": 101,
  "count": 4,
  "timestamp": "2023-04-28T14:10:00Z",
  "table_assignment": {
    "table_id": 301,
    "assigned_at": "2023-04-28T14:12:00Z",
    "released_at": null
  }
}
```

### 고객 그룹 수정

```
URL: /api/customers/{customer_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "count": 5
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "고객 그룹이 수정되었습니다."
}
```

### 고객 그룹 삭제

```
URL: /api/customers/{customer_id}
Method: DELETE
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "고객 그룹이 삭제되었습니다."
}
```

## 🏢 테이블 관리 API

### 테이블 목록 조회

```
URL: /api/tables
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 301,
    "table_number": 1,
    "max_customer": 4,
    "status": "OCCUPIED",
    "x": 10.5,
    "y": 20.3,
    "width": 2.0,
    "height": 2.0,
    "updated_at": "2023-04-28T14:12:00Z"
  },
  {
    "id": 302,
    "table_number": 2,
    "max_customer": 6,
    "status": "AVAILABLE",
    "x": 15.2,
    "y": 20.3,
    "width": 3.0,
    "height": 2.0,
    "updated_at": "2023-04-28T14:00:00Z"
  }
]
```

### 테이블 생성

```
URL: /api/tables
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "table_number": 3,
  "max_customer": 4,
  "x": 20.0,
  "y": 20.3,
  "width": 2.0,
  "height": 2.0
}
```

#### 응답 예시

```json
{
  "id": 303,
  "status": "success",
  "message": "테이블이 생성되었습니다."
}
```

### 테이블 상세 조회

```
URL: /api/tables/{table_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "id": 301,
  "table_number": 1,
  "max_customer": 4,
  "status": "OCCUPIED",
  "x": 10.5,
  "y": 20.3,
  "width": 2.0,
  "height": 2.0,
  "updated_at": "2023-04-28T14:12:00Z",
  "current_assignment": {
    "customer_id": 101,
    "assigned_at": "2023-04-28T14:12:00Z"
  }
}
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
  "status": "OCCUPIED"
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "테이블 상태가 업데이트되었습니다."
}
```

### 테이블 수정

```
URL: /api/tables/{table_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "max_customer": 6,
  "x": 11.0,
  "y": 21.0,
  "width": 3.0,
  "height": 2.5
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "테이블이 수정되었습니다."
}
```

### 테이블 삭제

```
URL: /api/tables/{table_id}
Method: DELETE
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "테이블이 삭제되었습니다."
}
```

## 🔄 테이블 배정 API

### 테이블 배정

```
URL: /api/tables/{table_id}/assign
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "customer_id": 101
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "테이블에 배정되었습니다."
}
```

### 배정 해제

```
URL: /api/tables/{table_id}/release
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "customer_id": 101
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "테이블 배정이 해제되었습니다."
}
```

### 테이블 배정 목록 조회

```
URL: /api/tables/assignments
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 401,
    "table_id": 301,
    "customer_id": 101,
    "timestamp": "2023-04-28T14:12:00Z",
    "released_at": null
  },
  {
    "id": 400,
    "table_id": 302,
    "customer_id": 100,
    "timestamp": "2023-04-28T13:30:00Z",
    "released_at": "2023-04-28T14:30:00Z"
  }
]
```

## 🖥️ 키오스크 API

### 키오스크 목록 조회

```
URL: /api/kiosks
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 401,
    "location": "Entrance",
    "ip_address": "192.168.0.20",
    "table_id": 301,
    "last_connected": "2023-04-28T15:42:07"
  },
  {
    "id": 402,
    "location": "Lobby",
    "ip_address": "192.168.0.21",
    "table_id": 302,
    "last_connected": "2023-04-28T15:40:23"
  }
]
```

### 키오스크 등록

```
URL: /api/kiosks
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "location": "Table 3",
  "ip_address": "192.168.0.22",
  "table_id": 303
}
```

#### 응답 예시

```json
{
  "id": 403,
  "status": "success",
  "message": "키오스크가 등록되었습니다."
}
```

### 키오스크 상세 조회

```
URL: /api/kiosks/{kiosk_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "id": 401,
  "location": "Entrance",
  "ip_address": "192.168.0.20",
  "table_id": 301,
  "last_connected": "2023-04-28T15:42:07"
}
```

### 키오스크 수정

```
URL: /api/kiosks/{kiosk_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "location": "Table 1",
  "table_id": 301
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "키오스크가 수정되었습니다."
}
```

### 키오스크 삭제

```
URL: /api/kiosks/{kiosk_id}
Method: DELETE
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "키오스크가 삭제되었습니다."
}
```

## 📝 주문 API

### 주문 목록 조회

```
URL: /api/orders
Method: GET
Authorization: Bearer {access_token}
Query Parameters:
  - status: (optional) 주문 상태 필터 (PLACED, PREPARING, SERVED, CANCELLED)
  - from_date: (optional) 시작 날짜 (YYYY-MM-DD)
  - to_date: (optional) 종료 날짜 (YYYY-MM-DD)
  - limit: (optional) 반환할 최대 주문 수
  - offset: (optional) 페이지네이션 오프셋
```

#### 응답 예시

```json
[
  {
    "id": 501,
    "customer_id": 101,
    "robot_id": 1,
    "table_id": 301,
    "status": "PREPARING",
    "timestamp": "2023-04-28T14:20:00Z",
    "served_at": null
  },
  {
    "id": 500,
    "customer_id": 100,
    "robot_id": 1,
    "table_id": 302,
    "status": "SERVED",
    "timestamp": "2023-04-28T13:50:00Z",
    "served_at": "2023-04-28T14:10:00Z"
  }
]
```

### 주문 상세 조회

```
URL: /api/orders/{order_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "id": 501,
  "customer_id": 101,
  "robot_id": 1,
  "table_id": 301,
  "status": "PREPARING",
  "timestamp": "2023-04-28T14:20:00Z",
  "served_at": null,
  "items": [
    {
      "menu_item_id": 601,
      "quantity": 2,
      "status": "PREPARING"
    },
    {
      "menu_item_id": 602,
      "quantity": 1,
      "status": "PREPARING"
    }
  ]
}
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
  "customer_id": 102,
  "table_id": 303,
  "items": [
    {
      "menu_item_id": 601,
      "quantity": 1
    },
    {
      "menu_item_id": 603,
      "quantity": 2
    }
  ]
}
```

#### 응답 예시

```json
{
  "id": 502,
  "status": "success",
  "message": "주문이 생성되었습니다."
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
  "status": "PREPARING"
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "주문 상태가 변경되었습니다."
}
```

### 주문 취소

```
URL: /api/orders/{order_id}/cancel
Method: POST
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "주문이 취소되었습니다."
}
```

## 🍽️ 메뉴 API

### 메뉴 항목 목록 조회

```
URL: /api/menu/items
Method: GET
Query Parameters:
  - is_available: (optional) 재고 여부 필터
```

#### 응답 예시

```json
[
  {
    "id": 601,
    "name": "스테이크",
    "price": 25.50,
    "prepare_time": 300,
    "image_url": "/images/steak.jpg",
    "description": "최고급 한우 스테이크"
  },
  {
    "id": 602,
    "name": "파스타",
    "price": 15.00,
    "prepare_time": 180,
    "image_url": "/images/pasta.jpg",
    "description": "토마토 소스의 파스타"
  }
]
```

### 메뉴 항목 상세 조회

```
URL: /api/menu/items/{item_id}
Method: GET
```

#### 응답 예시

```json
{
  "id": 601,
  "name": "스테이크",
  "price": 25.50,
  "prepare_time": 300,
  "image_url": "/images/steak.jpg",
  "description": "최고급 한우 스테이크",
  "ingredients": [
    {
      "id": 701,
      "name": "소고기",
      "quantity_required": 2
    },
    {
      "id": 702,
      "name": "버터",
      "quantity_required": 1
    }
  ]
}
```

### 메뉴 항목 생성

```
URL: /api/menu/items
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "name": "돈까스",
  "price": 12.00,
  "prepare_time": 180,
  "image_url": "/images/pork_cutlet.jpg",
  "description": "바삭한 돈까스"
}
```

#### 응답 예시

```json
{
  "id": 604,
  "status": "success",
  "message": "메뉴 항목이 생성되었습니다."
}
```

### 메뉴 항목 수정

```
URL: /api/menu/items/{item_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "price": 13.00,
  "prepare_time": 200,
  "description": "더 바삭한 돈까스"
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "메뉴 항목이 수정되었습니다."
}
```

### 메뉴 항목 삭제

```
URL: /api/menu/items/{item_id}
Method: DELETE
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "메뉴 항목이 삭제되었습니다."
}
```

## 🥩 재료 관리 API

### 재료 목록 조회

```
URL: /api/menu/ingredients
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 701,
    "name": "소고기",
    "menu_item_id": 601,
    "quantity_required": 2
  },
  {
    "id": 702,
    "name": "버터",
    "menu_item_id": 601,
    "quantity_required": 1
  }
]
```

### 재료 상세 조회

```
URL: /api/menu/ingredients/{ingredient_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "id": 701,
  "name": "소고기",
  "menu_item_id": 601,
  "quantity_required": 2
}
```

### 재료 생성

```
URL: /api/menu/ingredients
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "name": "치즈",
  "menu_item_id": 602,
  "quantity_required": 1
}
```

#### 응답 예시

```json
{
  "id": 703,
  "status": "success",
  "message": "재료가 생성되었습니다."
}
```

### 재료 수정

```
URL: /api/menu/ingredients/{ingredient_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "quantity_required": 3
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "재료가 수정되었습니다."
}
```

### 재료 삭제

```
URL: /api/menu/ingredients/{ingredient_id}
Method: DELETE
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "재료가 삭제되었습니다."
}
```

## 📦 재고 관리 API

### 재고 목록 조회

```
URL: /api/inventory
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 801,
    "ingredient_id": 701,
    "name": "소고기",
    "count": 50,
    "max_count": 100,
    "status": "IN_STOCK",
    "updated_at": "2023-04-28T10:00:00Z"
  },
  {
    "id": 802,
    "ingredient_id": 702,
    "name": "버터",
    "count": 5,
    "max_count": 50,
    "status": "LOW_STOCK",
    "updated_at": "2023-04-28T10:05:00Z"
  }
]
```

### 재고 상세 조회

```
URL: /api/inventory/{inventory_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "id": 801,
  "ingredient_id": 701,
  "name": "소고기",
  "count": 50,
  "max_count": 100,
  "status": "IN_STOCK",
  "updated_at": "2023-04-28T10:00:00Z"
}
```

### 재고 생성

```
URL: /api/inventory
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "ingredient_id": 703,
  "name": "치즈",
  "count": 30,
  "max_count": 60
}
```

#### 응답 예시

```json
{
  "id": 803,
  "status": "success",
  "message": "재고가 생성되었습니다."
}
```

### 재고 수정

```
URL: /api/inventory/{inventory_id}
Method: PUT
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "count": 25,
  "max_count": 60
}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "재고가 업데이트되었습니다."
}
```

### 재고 삭제

```
URL: /api/inventory/{inventory_id}
Method: DELETE
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "재고가 삭제되었습니다."
}
```

## 📊 이벤트 관리 API

### 이벤트 목록 조회

```
URL: /api/events
Method: GET
Authorization: Bearer {access_token}
Query Parameters:
  - type: (optional) 이벤트 타입 (WELCOME, CALL, BIRTHDAY, EMERGENCY, CLEANING)
  - entity_type: (optional) 관련 엔티티 타입 (ALBABOT, COOKBOT, INVENTORY, CHATBOT)
  - limit: (optional) 반환할 최대 이벤트 수
  - offset: (optional) 페이지네이션 오프셋
```

#### 응답 예시

```json
[
  {
    "id": 901,
    "type": "WELCOME",
    "related_entity_type": "ALBABOT",
    "related_entity_id": 1,
    "description": "환영 이벤트",
    "timestamp": "2023-04-28T12:00:00Z"
  },
  {
    "id": 902,
    "type": "CALL",
    "related_entity_type": "CHATBOT",
    "related_entity_id": "c101",
    "description": "고객 호출 이벤트",
    "timestamp": "2023-04-28T12:05:00Z"
  }
]
```

### 이벤트 상세 조회

```
URL: /api/events/{event_id}
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "id": 901,
  "type": "WELCOME",
  "related_entity_type": "ALBABOT",
  "related_entity_id": 1,
  "description": "환영 이벤트",
  "timestamp": "2023-04-28T12:00:00Z"
}
```

### 이벤트 생성

```
URL: /api/events
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "type": "BIRTHDAY",
  "related_entity_type": "ALBABOT",
  "related_entity_id": 1,
  "description": "생일 축하 이벤트"
}
```

#### 응답 예시

```json
{
  "id": 903,
  "status": "success",
  "message": "이벤트가 생성되었습니다."
}
```

### 이벤트 삭제

```
URL: /api/events/{event_id}
Method: DELETE
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "이벤트가 삭제되었습니다."
}
```

### 시스템 로그 조회

```
URL: /api/system-logs
Method: GET
Authorization: Bearer {access_token}
Query Parameters:
  - level: (optional) 로그 레벨 (INFO, WARNING, ERROR, CRITICAL)
  - limit: (optional) 반환할 최대 로그 수
  - offset: (optional) 페이지네이션 오프셋
```

#### 응답 예시

```json
[
  {
    "id": 1001,
    "level": "ERROR",
    "message": "서버 오류 발생",
    "timestamp": "2023-04-28T13:00:00Z"
  },
  {
    "id": 1002,
    "level": "WARNING",
    "message": "재고 부족 경고",
    "timestamp": "2023-04-28T13:05:00Z"
  }
]
```

## 👁️ 얼굴 인식 API

### 얼굴 인식 데이터 목록 조회

```
URL: /api/face-recognitions
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 1,
    "table_id": 301,
    "timestamp": "2023-04-28T14:20:00Z",
    "history": "[0, 0, 1, 1, 1]",
    "nowdetected": 3,
    "reliability": 85,
    "exist": 1
  },
  {
    "id": 2,
    "table_id": 302,
    "timestamp": "2023-04-28T14:25:00Z",
    "history": "[0, 1, 0, 0, 0]",
    "nowdetected": 1,
    "reliability": 30,
    "exist": 0
  }
]
```

### 얼굴 인식 데이터 생성

```
URL: /api/face-recognitions
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "table_id": 301,
  "history": "[0, 1, 1, 1, 1]",
  "nowdetected": 4,
  "reliability": 90,
  "exist": 1
}
```

#### 응답 예시

```json
{
  "id": 3,
  "table_id": 301,
  "timestamp": "2023-04-28T14:30:00Z",
  "history": "[0, 1, 1, 1, 1]",
  "nowdetected": 4,
  "reliability": 90,
  "exist": 1
}
```

## 🚨 비상 상황 관리 API

### 비상 상황 목록 조회

```
URL: /api/emergencies
Method: GET
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
[
  {
    "id": 1,
    "emergency_type": "FIRE",
    "description": "주방에서 화재 발생",
    "is_active": true,
    "reported_at": "2023-04-28T12:30:00Z",
    "resolved_at": null
  },
  {
    "id": 2,
    "emergency_type": "MEDICAL",
    "description": "테이블 3에서 손님 응급상황",
    "is_active": false,
    "reported_at": "2023-04-28T11:45:00Z",
    "resolved_at": "2023-04-28T12:15:00Z"
  }
]
```

### 비상 상황 보고

```
URL: /api/emergencies
Method: POST
Authorization: Bearer {access_token}
Content-Type: application/json
```

#### 요청 예시

```json
{
  "emergency_type": "ROBOT_MALFUNCTION",
  "description": "요리 로봇 작동 불량"
}
```

#### 응답 예시

```json
{
  "id": 3,
  "status": "success",
  "message": "비상 상황이 신고되었습니다."
}
```

### 비상 상황 해제

```
URL: /api/emergencies/{emergency_id}/resolve
Method: PUT
Authorization: Bearer {access_token}
```

#### 응답 예시

```json
{
  "status": "success",
  "message": "비상 상황이 해제되었습니다."
}
``` 