# 🌐 RoboDine 인터페이스 명세서

## 📌 개요

이 문서는 RoboDine 서비스의 인터페이스 명세를 제공합니다. 여기에는 API 엔드포인트, TCP/UDP 통신 프로토콜, WebSocket 명세, 그리고 데이터 모델 정의가 포함되어 있습니다.

## 🔹 Enum 정의

### 기본 열거형 타입

```python
class EntityType(str, Enum):
    COOKBOT = "COOKBOT"     # 조리 로봇
    ALBABOT = "ALBABOT"     # 서빙 로봇
    PINKY = "PINKY"        # 위치 좌표계 (서빙 로봇 내부)
    GLOBAL = "GLOBAL"      # 위치 좌표계 (글로벌 좌표)
    WORLD = "WORLD"        # 위치 좌표계 (월드 좌표)
    INVENTORY = "INVENTORY" # 재고 아이템
    CHATBOT = "CHATBOT"    # 챗봇

class RobotStatus(str, Enum):
    IDLE = "IDLE"           # 대기 상태
    SETTING = "SETTING"     # 설정 중
    COOKING = "COOKING"     # 조리 중
    PICKUP = "PICKUP"       # 픽업 중
    SERVING = "SERVING"     # 서빙 중
    CLEANING = "CLEANING"   # 청소 중
    EMERGENCY = "EMERGENCY" # 비상 상태
    MAINTENANCE = "MAINTENANCE" # 유지보수 중
    SECURITY = "SECURITY"   # 보안 모드

class CommandStatus(str, Enum):
    PENDING = "PENDING"     # 대기 중
    SENT = "SENT"           # 전송됨
    ACKED = "ACKED"         # 수신 확인됨
    EXECUTED = "EXECUTED"   # 실행 완료
    FAILED = "FAILED"       # 실패

class InventoryStatus(str, Enum):
    IN_STOCK = "IN_STOCK"       # 재고 있음
    LOW_STOCK = "LOW_STOCK"     # 재고 부족
    OUT_OF_STOCK = "OUT_OF_STOCK" # 재고 없음

class TableStatus(str, Enum):
    AVAILABLE = "AVAILABLE" # 사용 가능
    OCCUPIED = "OCCUPIED"   # 사용 중

class OrderStatus(str, Enum):
    PLACED = "PLACED"       # 주문 접수됨
    PREPARING = "PREPARING" # 준비 중
    SERVED = "SERVED"       # 서빙 완료
    CANCELLED = "CANCELLED" # 취소됨

class EventType(str, Enum):
    WELCOME = "WELCOME"     # 환영 이벤트
    CALL = "CALL"           # 직원 호출
    BIRTHDAY = "BIRTHDAY"   # 생일 축하
    EMERGENCY = "EMERGENCY" # 비상 상황
    CLEANING = "CLEANING"   # 청소 요청

class StreamSourceType(str, Enum):
    ALBABOT = "ALBABOT"     # 서빙 로봇 카메라
    COOKBOT = "COOKBOT"     # 조리 로봇 카메라
    GLOBAL_CAM = "GLOBAL_CAM" # 공간 모니터링 카메라

class StreamStatus(str, Enum):
    ACTIVE = "ACTIVE"       # 활성 상태
    INACTIVE = "INACTIVE"   # 비활성 상태
    ERROR = "ERROR"         # 오류 상태

class UserRole(str, Enum):
    ADMIN = "ADMIN"         # 관리자
    KIOSK = "KIOSK"         # 키오스크

class NotificationStatus(str, Enum):
    PENDING = "PENDING"     # 대기 중
    SENT = "SENT"           # 전송됨
    FAILED = "FAILED"       # 실패
```

## 🔹 TCP 통신 명세

### 📍 Alba Manager to Robodine Service

#### 송신 데이터 (Request)

```json
{
    "msg_type": "Albabot",
    "robot_id": 1,
    "status": "IDLE",
    "battery_level": 85,
    "pinky_x": 10.5,
    "pinky_y": 20.3,
    "pinky_z": 0.0,
    "pinky_roll": 0.0,
    "pinky_pitch": 0.0,
    "pinky_yaw": 90.0,
    "global_x": 100.5,
    "global_y": 200.3,
    "global_z": 0.0,
    "global_roll": 0.0,
    "global_pitch": 0.0,
    "global_yaw": 90.0,
    "world_x": 1000.5,
    "world_y": 2000.3,
    "world_z": 0.0,
    "world_roll": 0.0,
    "world_pitch": 0.0,
    "world_yaw": 90.0
}
```

#### 수신 데이터 (Response)

```json
{
  "status": "success",
  "message": "정보가 성공적으로 등록되었습니다."
}
```

#### 데이터 처리 예시

```python
# Albabot 클래스
{
    "robot_id": robot_id,
    "battery_level": battery_level,
    "status": status,
    "timestamp": now()
}

# Pose6D 클래스 (PINKY 좌표계)
{
    "entity_id": robot_id,
    "entity_type": "PINKY",
    "timestamp": now(),
    "x": pinky_x,
    "y": pinky_y,
    "z": pinky_z,
    "roll": pinky_roll,
    "pitch": pinky_pitch,
    "yaw": pinky_yaw
}

# Pose6D 클래스 (GLOBAL 좌표계)
{
    "entity_id": robot_id,
    "entity_type": "GLOBAL",
    "timestamp": now(),
    "x": global_x,
    "y": global_y,
    "z": global_z,
    "roll": global_roll,
    "pitch": global_pitch,
    "yaw": global_yaw
}

# Pose6D 클래스 (WORLD 좌표계)
{
    "entity_id": robot_id,
    "entity_type": "WORLD",
    "timestamp": now(),
    "x": world_x,
    "y": world_y,
    "z": world_z,
    "roll": world_roll,
    "pitch": world_pitch,
    "yaw": world_yaw
}
```

### 📍 Cook Manager to Robodine Service

#### 송신 데이터 (Request)

```json
{
    "msg_type": "Cookbot",
    "robot_id": 2,
    "status": "COOKING",
    "angle_1": 45.0,
    "angle_2": 90.0,
    "angle_3": 30.0,
    "angle_4": 0.0,
    "angle_5": 60.0,
    "angle_6": 10.0,
    "endpoint_x": 0.5,
    "endpoint_y": 0.3,
    "endpoint_z": 0.2,
    "endpoint_roll": 0.0,
    "endpoint_pitch": 30.0,
    "endpoint_yaw": 0.0
}
```

#### 수신 데이터 (Response)

```json
{
  "status": "success",
  "message": "정보가 성공적으로 등록되었습니다."
}
```

#### 데이터 처리 예시

```python
# Cookbot 클래스
{
    "robot_id": robot_id,
    "status": status,
    "timestamp": now()
}

# Pose6D 클래스 (COOKBOT 좌표계)
{
    "entity_id": robot_id,
    "entity_type": "COOKBOT",
    "timestamp": now(),
    "x": endpoint_x,
    "y": endpoint_y,
    "z": endpoint_z,
    "roll": endpoint_roll,
    "pitch": endpoint_pitch,
    "yaw": endpoint_yaw
}

# JointAngle 클래스
{
    "robot_id": robot_id,
    "timestamp": now(),
    "joint_1": angle_1,
    "joint_2": angle_2,
    "joint_3": angle_3,
    "joint_4": angle_4,
    "joint_5": angle_5,
    "joint_6": angle_6
}
```

### 📍 Cook GPT to Robodine Service

#### 송신 데이터 (Request)

```json
{
    "msg_type": "Ingredient",
    "ingredient_id": 101,
    "x": 0.5,
    "y": 0.3,
    "z": 0.1,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 45.0
}
```

#### 수신 데이터 (Response)

```json
{
  "status": "success",
  "message": "정보가 성공적으로 등록되었습니다."
}
```

#### 데이터 처리 예시

```python
# Pose6D 클래스 (INVENTORY 좌표계)
{
    "entity_id": ingredient_id,
    "entity_type": "INVENTORY",
    "timestamp": now(),
    "x": x,
    "y": y,
    "z": z,
    "roll": roll,
    "pitch": pitch,
    "yaw": yaw
}
```

## 🔹 REST API 명세

### A. 공통 API

| 기능 | 메서드 | 경로 |
|------|------|------|
| 로봇 등록 | POST | /robots/register |
| 로봇 상세 조회 | GET | /robots/{robot_id} |
| 로봇 조회 | GET | /robots |
| 명령 발행 | POST | /commands/{robot_id}/command |
| 명령 상태 변경 | PUT | /commands/{command_id}/status |
| 명령 목록 조회 | GET | /commands/{robot_id} |

### B. Albabot 전용 API

| 기능 | 메서드 | 경로 |
|------|------|------|
| 상태·배터리 조회 | GET | /albabot/status/{robot_id} |

### C. Cookbot 전용 API

| 기능 | 메서드 | 경로 |
|------|------|------|
| 상태 조회 | GET | /cookbot/status/{robot_id} |
| 준비해야하는 메뉴 조회 | GET | /todo_order |

### D. 사용자 API

| 기능 | 메서드 | 경로 |
|------|------|------|
| 로그인 | POST | /auth/login |
| 로그아웃 | POST | /auth/logout |
| 목록 조회 | GET | /users |
| 상세 조회 | GET | /users/{user_id} |
| 생성 | POST | /users |
| 수정 | PUT | /users/{user_id} |

### E. 알림 API

| 기능 | 메서드 | 경로 |
|------|------|------|
| 목록 조회 | GET | /users/{user_id}/notifications |
| 생성 | POST | /users/{user_id}/notifications |

### F. 관리자 설정 API

| 기능 | 메서드 | 경로 |
|------|------|------|
| 조회 | GET | /settings |
| 업데이트 | PUT | /settings |

### G. 고객·테이블·대기열·배정 API

| 기능 | 메서드 | 경로 |
|------|------|------|
| 고객 그룹 목록 | GET | /customers |
| 고객 그룹 생성 | POST | /customers |
| 테이블 목록 | GET | /tables |
| 테이블 등록 | POST | /tables |
| 대기열 목록 | GET | /waiting-list |
| 대기 등록 | POST | /waiting-list |
| 대기 해제 | DELETE | /waiting-list/{id} |
| 배정 | POST | /tables/{table_id}/assign |
| 배정 해제 | POST | /tables/{table_id}/release |

### H. 키오스크 API

| 기능 | 메서드 | 경로 |
|------|------|------|
| 목록 조회 | GET | /kiosks |
| 등록 | POST | /kiosks |

### I. 주문 API

| 기능 | 메서드 | 경로 |
|------|------|------|
| 주문 목록 | GET | /orders |
| 주문 상세 | GET | /orders/{order_id} |
| 주문 생성 | POST | /orders |
| 상태 변경 | PUT | /orders/{order_id}/status |

### J. 메뉴·재료 API

| 기능 | 메서드 | 경로 |
|------|------|------|
| 메뉴 목록 | GET | /menu/items |
| 메뉴 상세 | GET | /menu/items/{item_id} |
| 메뉴 생성 | POST | /menu/items |
| 메뉴 수정 | PUT | /menu/items/{item_id} |
| 메뉴 삭제 | DELETE | /menu/items/{item_id} |
| 재료 목록 | GET | /menu/ingredients |
| 재료 상세 | GET | /menu/ingredients/{ingredient_id} |
| 재료 생성 | POST | /menu/ingredients |
| 재료 수정 | PUT | /menu/ingredients/{ingredient_id} |
| 재료 삭제 | DELETE | /menu/ingredients/{ingredient_id} |
| 재고 목록 | GET | /inventory |
| 재고 생성 | POST | /inventory |
| 재고 수정 | PUT | /inventory/{inventory_id} |
| 재고 삭제 | DELETE | /inventory/{inventory_id} |

### K. 이벤트·로그·비디오·비상 API

| 기능 | 메서드 | 경로 | 설명 |
|------|------|------|------|
| 이벤트 목록 | GET | /events | 로그 조회 |
| 이벤트 생성 | POST | /events | 신규 이벤트 기록 |
| 이벤트 삭제 | DELETE | /events/{event_id} | |
| 시스템 로그 조회 | GET | /system-logs | 전체 또는 ?level=&limit= |
| 시스템 로그 생성 | POST | /system-logs | |
| 비디오 스트림 | GET | /video-streams | |
| 스트림 갱신 | PUT | /video-streams/{stream_id}/refresh | URL 업데이트 |
| 비상 목록 | GET | /emergencies | |
| 비상 신고 | POST | /emergencies | |
| 비상 해제 | PUT | /emergencies/{emergency_id}/resolve | |

## 🔹 API 예시

### 📍 로봇 공통 API

#### 로봇 MAC-IP 등록
**Endpoint**: POST /robot/register

**Request**:
```json
{
  "robot_id": 1,
  "robot_type": "ALBABOT",
  "mac_address": "AA:BB:CC:11:22:33",
  "ip": "192.168.0.10"
}
```

**Response**:
```json
{
  "robot_id": 1,
  "status": "success",
  "message": "로봇 정보가 성공적으로 등록되었습니다."
}
```

#### 로봇 MAC-IP 조회
**Endpoint**: GET /robot/{robot_id}

**Response**:
```json
{
  "robot_id": 1,
  "robot_type": "ALBABOT",
  "mac_address": "AA:BB:CC:11:22:33",
  "ip_address": "192.168.0.10",
  "timestamp": "2025-04-28 11:42:07"
}
```

#### Albabot 명령 요청
**Endpoint**: POST /commands/{robot_id}/command

**Request**:
```json
{
  "robot_id": 1,
  "command": "Move",
  "parameter": {"x": 5.0, "y": 3.0, "z": 0.0}
}
```

**Response**:
```json
{
  "id": 24,
  "status": "success",
  "message": "명령이 성공적으로 전송되었습니다."
}
```

#### 로봇 명령 상태 변경
**Endpoint**: PUT /commands/{command_id}/status

**Request**:
```json
{
  "id": 24,
  "status": "EXECUTED"
}
```

**Response**:
```json
{
  "status": "success",
  "message": "명령의 상태가 성공적으로 변경되었습니다."
}
```

#### 로봇 명령 상태 조회
**Endpoint**: GET /commands/{robot_id}

**Response**:
```json
{
  "id": 24,
  "command": "Moving",
  "status": "EXECUTED"
}
```

### 📍 Albabot (Pinky) 전용 API

#### Albabot 상태 및 배터리 조회
**Endpoint**: GET /albabot/status/{robot_id}

**Response**:
```json
{
  "robot_id": 1,
  "status": "IDLE",
  "battery_level": "95",
  "timestamp": "2025-04-28 11:42:07"
}
```

### 📍 Cookbot (JetCobot) 전용 API

#### Cookbot 상태 조회
**Endpoint**: GET /cookbot/status/{robot_id}

**Response**:
```json
{
  "robot_id": 1,
  "status": "COOKING",
  "timestamp": "2025-04-28 11:42:07"
}
```

### 📍 사용자(USER) API

#### 로그인
**Endpoint**: POST /auth/login

**Request**:
```json
{
  "username": "user1",
  "password": "pass123"
}
```

**Response**:
```json
{
  "access_token": "eyJ...",
  "token_type": "Bearer"
}
```

#### 로그아웃
**Endpoint**: POST /auth/logout

**Response**:
```json
{
  "status": "success",
  "message": "로그아웃 완료."
}
```

#### 사용자 목록 조회
**Endpoint**: GET /users

**Response**:
```json
[
  {
    "id": 1,
    "username": "admin",
    "role": "ADMIN",
    "created_at": "2025-01-01T09:00:00Z",
    "updated_at": "2025-03-10T12:00:00Z"
  },
  ...
]
```

#### 특정 사용자 조회
**Endpoint**: GET /users/{user_id}

**Response**:
```json
{
  "id": 2,
  "username": "kiosk1",
  "role": "KIOSK",
  "created_at": "2025-04-28T15:00:00Z",
  "updated_at": "2025-04-28T15:00:00Z"
}
```

#### 사용자 생성
**Endpoint**: POST /users

**Request**:
```json
{
  "username": "newuser",
  "password": "pass123",
  "role": "KIOSK"
}
```

**Response**:
```json
{
  "id": 3,
  "username": "newuser",
  "role": "KIOSK",
  "created_at": "2025-04-28T16:00:00Z",
  "updated_at": "2025-04-28T16:00:00Z"
}
```

#### 사용자 수정
**Endpoint**: PUT /users/{user_id}

**Request**:
```json
{
  "password": "newpass",
  "role": "ADMIN"
}
```

**Response**:
```json
{
  "status": "success",
  "message": "사용자 정보가 성공적으로 수정되었습니다."
}
```

### 📍 알림(Notification) API

#### 알림 목록 조회
**Endpoint**: GET /users/{user_id}/notifications

**Response**:
```json
[
  {
    "id": 10,
    "type": "ORDER_STATUS",
    "message": "주문이 준비되었습니다.",
    "created_at": "2025-04-28T14:00:00Z",
    "status": "PENDING"
  },
  ...
]
```

#### 알림 생성
**Endpoint**: POST /users/{user_id}/notifications

**Request**:
```json
{
  "type": "CALL",
  "message": "도움이 필요합니다."
}
```

**Response**:
```json
{
  "id": 11,
  "status": "success",
  "message": "알림이 생성되었습니다."
}
```

### 📍 관리자 설정(AdminSettings) API

#### 설정 조회
**Endpoint**: GET /settings

**Response**:
```json
{
  "operation_start": "08:00",
  "operation_end": "22:00",
  "inventory_threshold": 10,
  "alert_settings": {"low_stock": true}
}
```

#### 설정 업데이트
**Endpoint**: PUT /settings

**Request**:
```json
{
  "operation_start": "09:00",
  "operation_end": "21:00",
  "inventory_threshold": 5
}
```

**Response**:
```json
{
  "status": "success",
  "message": "설정이 업데이트되었습니다."
}
```

### 📍 고객(Customer) API

#### 고객 목록 조회
**Endpoint**: GET /customers

**Response**:
```json
[
  {"id": 101, "count": 4, "created_at": "2025-04-28T14:10:00Z"},
  ...
]
```

#### 고객 그룹 생성
**Endpoint**: POST /customers

**Request**:
```json
{ "count": 3 }
```

**Response**:
```json
{ "id": 102, "count": 3, "created_at": "2025-04-28T16:10:00Z" }
```

### 📍 테이블(Table) API

#### 테이블 목록 조회
**Endpoint**: GET /tables

**Response**:
```json
[
  {"id":301, "table_number":1, "max_customer":4, "status":"AVAILABLE"},
  ...
]
```

#### 테이블 등록
**Endpoint**: POST /tables

**Request**:
```json
{ "number": 2, "max_customer": 4 }
```

**Response**:
```json
{ "id":302, "status":"success" }
```

### 📍 배정(GroupAssignment) API

#### 그룹 테이블 배정
**Endpoint**: POST /tables/{table_id}/assign

**Request**:
```json
{ "customer_id": 101 }
```

**Response**:
```json
{
  "status":"success", 
  "message":"테이블에 배정되었습니다." 
}
```

#### 그룹 테이블 해제
**Endpoint**: POST /tables/{table_id}/release

**Response**:
```json
{ "status":"success", "message":"테이블 배정이 해제되었습니다." }
```

### 📍 키오스크(KioskTerminal) API

#### 키오스크 목록 조회
**Endpoint**: GET /kiosks

**Response**:
```json
[
  {"id":401, "location":"Entrance", "ip_address":"192.168.0.20"},
  ...
]
```

#### 키오스크 등록
**Endpoint**: POST /kiosks

**Request**:
```json
{ "location": "Lobby", "ip_address": "192.168.0.21" }
```

**Response**:
```json
{ "id":402, "status":"success" }
```

### 📍 주문(Order) API

#### 주문 목록 조회
**Endpoint**: GET /orders

**Response**:
```json
[
  {"id":501, "customer_id":101, "status":"PLACED", "ordered_at":"2025-04-28T14:20:00Z"},
  ...
]
```

#### 주문 상세 조회
**Endpoint**: GET /orders/{order_id}

**Response**:
```json
{
  "id":501,
  "customer_id":101,
  "items":[{"menu_item_id":601,"quantity":2}],
  "status":"PLACED",
  "ordered_at":"2025-04-28T14:20:00Z",
  "served_at":null
}
```

#### 주문 생성
**Endpoint**: POST /orders

**Request**:
```json
{ "customer_id":102, "items":[{"menu_item_id":601,"quantity":1}] }
```

**Response**:
```json
{ "id":502, "status":"success" }
```

#### 주문 상태 변경
**Endpoint**: PUT /orders/{order_id}/status

**Request**:
```json
{ "status": "PREPARING" }
```

**Response**:
```json
{ "status":"success", "message":"주문 상태가 변경되었습니다." }
```

### 📍 메뉴(MenuItem) API

#### 메뉴 항목 목록 조회
**Endpoint**: GET /menu/items

**Response**:
```json
[
  {"id": 601, "name": "스테이크", "price": 25.50, "prepare_time": 300},
  ...
]
```

#### 메뉴 항목 상세 조회
**Endpoint**: GET /menu/items/{item_id}

**Response**:
```json
{
  "id": 601,
  "name": "스테이크",
  "price": 25.50,
  "prepare_time": 300,
  "menu_ingredients": [
    {"ingredient_id": 701, "quantity_required": 2},
    ...
  ]
}
```

#### 메뉴 항목 생성
**Endpoint**: POST /menu/items

**Request**:
```json
{
  "name": "돈까스",
  "price": 12.00,
  "prepare_time": 180
}
```

**Response**:
```json
{
  "id": 602,
  "status": "success",
  "message": "메뉴 항목이 생성되었습니다."
}
```

### 📍 재료(MenuIngredient) API

#### 재료 목록 조회
**Endpoint**: GET /menu/ingredients

**Response**:
```json
[
  {"id": 701, "name": "소고기", "menu_item_id": 601, "quantity_required": 2},
  ...
]
```

#### 재료 상세 조회
**Endpoint**: GET /menu/ingredients/{ingredient_id}

**Response**:
```json
{
  "id": 701,
  "name": "소고기",
  "menu_item_id": 601,
  "quantity_required": 2
}
```

### 📍 재고(Inventory) API

#### 재고 목록 조회
**Endpoint**: GET /inventory

**Response**:
```json
{
  "ingredient_id": 701,
  "name": "소고기",
  "count": 50,
  "status": "IN_STOCK"
}
```

### 📍 이벤트(Event) API

#### 이벤트 목록 조회
**Endpoint**: GET /events

**Response**:
```json
[
  {"id": 801, "type": "WELCOME", "related_entity_type": "PINKY", "related_entity_id": "123", "description": "환영 이벤트", "timestamp": "2025-04-28T12:00:00Z"},
  ...
]
```

#### 이벤트 생성
**Endpoint**: POST /events

**Request**:
```json
{
  "type": "CALL",
  "related_entity_type": "CHATBOT",
  "related_entity_id": "c101",
  "description": "고객 호출 이벤트"
}
```

**Response**:
```json
{
  "id": 802,
  "status": "success",
  "message": "이벤트가 생성되었습니다."
}
```

### 📍 비디오 스트림(VideoStream) API

#### 스트림 목록 조회
**Endpoint**: GET /video-streams

**Response**:
```json
[
  {"id": 1001, "source_type": "GLOBAL_CAM", "source_id": "cam01", "url": "rtsp://...", "last_checked": "2025-04-28T14:00:00Z", "status": "ACTIVE"},
  ...
]
```

#### 스트림 URL 갱신
**Endpoint**: PUT /video-streams/{stream_id}/refresh

**Request**:
```json
{ "url": "rtsp://new_url" }
```

**Response**:
```json
{ "status": "success", "message": "스트림 URL이 갱신되었습니다." }
```

### 📍 비상 상황(Emergency) API

#### 비상 목록 조회
**Endpoint**: GET /emergencies

**Response**:
```json
[
  {"id": 1101, "emergency_type": "FIRE", "description": "화재 감지", "is_active": true, "reported_at": "2025-04-28T15:00:00Z", "resolved_at": null},
  ...
]
```

#### 비상 신고
**Endpoint**: POST /emergencies

**Request**:
```json
{ "emergency_type": "MEDICAL", "description": "응급상황 발생" }
```

**Response**:
```json
{ "id": 1102, "status": "success", "message": "비상 상황이 신고되었습니다." }
```

#### 비상 해제
**Endpoint**: PUT /emergencies/{emergency_id}/resolve

**Response**:
```json
{ "status": "success", "message": "비상 상황이 해제되었습니다." }
```

## 🔹 WebSocket 명세

### 📍 실시간 로봇 상태 및 테이블 상태 업데이트

**Endpoint**: /ws/status

**메시지 형식**:
```json
{
  "event": "status_update",
  "data": {
    "robot_id": "1",
    "status": "조리 완료",
    "location": {"x": 1.0, "y": 2.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 90.0},
    "table_id": "1",
    "customers": [
      {"customer_id": "1", "count": "1", "food": "스테이크", "drink": "와인", "status": "식사 중"}
    ]
  }
}
```

## 📝 주석 및 설명

1. **고객 입장 정책**: 고객은 무조건 한 명만 입장합니다.

2. **이벤트 옵션**: 주문 시 생일 이벤트나 알바 호출 여부를 옵션으로 설정할 수 있습니다.

3. **위치 정보**: 
   - Pinky 로봇의 location은 6D 좌표를 사용하여 웹에서 미니맵 형태로 위치를 시각화합니다.
   - MyCobot280 로봇은 joint_positions을 통해 로봇팔 상태를 실시간으로 전달합니다.

4. **통신 프로토콜**:
   - API 응답은 JSON 형식이며, 상태 및 에러는 HTTP 상태 코드를 활용하여 처리합니다.
   - WebSocket을 통해 로봇 및 테이블 상태를 실시간으로 프론트엔드에 제공합니다.

5. **보안 정책**:
   - 모든 API 엔드포인트는 인증이 필요합니다 (일부 공개 엔드포인트 제외).
   - JWT를 통한 토큰 기반 인증이 사용됩니다.

6. **오류 처리**:
   - 모든 API 응답은 일관된 형식을 유지하며, 오류 발생 시 적절한 HTTP 상태 코드와 메시지가 반환됩니다.
   - 시스템 로그에 모든 오류가 기록됩니다. 