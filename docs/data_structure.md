# Robodine Service 데이터 구조

본 문서는 Robodine Service 백엔드 시스템의 데이터 구조를 설명합니다. 이 문서는 `app/models` 디렉토리에 정의된 모델을 기반으로 작성되었습니다.

## 목차

- [열거형 (Enums)](#열거형-enums)
- [로봇 관련 모델](#로봇-관련-모델)
  - [Robot](#robot)
  - [Cookbot](#cookbot)
  - [Albabot](#albabot)
  - [RobotCommand](#robotcommand)
  - [Pose6D](#pose6d)
  - [JointAngle](#jointangle)
- [주문 관련 모델](#주문-관련-모델)
  - [Order](#order)
  - [OrderItem](#orderitem)
  - [KioskTerminal](#kioskterminal)
- [재고 관련 모델](#재고-관련-모델)
  - [Inventory](#inventory)
  - [MenuItem](#menuitem)
  - [MenuIngredient](#menuingredient)
- [테이블 관련 모델](#테이블-관련-모델)
  - [Table](#table)
  - [GroupAssignment](#groupassignment)
- [고객 관련 모델](#고객-관련-모델)
  - [Customer](#customer)
  - [FaceRecognition](#facerecognition)
- [사용자 관련 모델](#사용자-관련-모델)
  - [User](#user)
  - [Notification](#notification)
- [시스템 관련 모델](#시스템-관련-모델)
  - [AdminSettings](#adminsettings)
  - [Event](#event)
  - [SystemLog](#systemlog)
  - [Emergency](#emergency)
- [미디어 관련 모델](#미디어-관련-모델)
  - [VideoStream](#videostream)
  - [Chat](#chat)
- [청소 관련 모델](#청소-관련-모델)
  - [CleaningTask](#cleaningtask)

## 열거형 (Enums)

시스템 전반에서 사용되는 열거형들입니다:

| 열거형 이름 | 설명 | 값 |
|------------|------|-----|
| ChatType | 채팅 메시지 유형 | TEXT, IMAGE, VOICE, ACTION |
| ChatStatus | 채팅 상태 | PENDING, COMPLETED, ERROR, RETRYING, SENT |
| TaskType | 작업 유형 | GREETINGS, MAINTENANCE, TAKE_PICTURE |
| LogLevel | 로그 레벨 | INFO, WARNING, ERROR, DEBUG |
| EntityType | 엔티티 유형 | COOKBOT, ALBABOT, PINKY, GLOBAL, WORLD, INVENTORY, CHATBOT |
| RobotStatus | 로봇 상태 | IDLE, SETTING, COOKING, PICKUP, SERVING, CLEANING, EMERGENCY, MAINTENANCE, SECURITY, BIRTHDAY, CALLING, ERROR |
| CommandStatus | 명령 상태 | PENDING, SENT, EXECUTING, EXECUTED, FAILED, CANCELLED |
| InventoryStatus | 재고 상태 | IN_STOCK, LOW_STOCK, OUT_OF_STOCK |
| TableStatus | 테이블 상태 | AVAILABLE, OCCUPIED, CLEANING |
| OrderStatus | 주문 상태 | PLACED, PREPARING, COMPLETED, SERVED, CANCELLED |
| EventType | 이벤트 유형 | WELCOME, CALL, BIRTHDAY, EMERGENCY, CLEANING |
| StreamSourceType | 스트림 소스 유형 | PINKY, COOKBOT, GLOBAL_CAM, WEBCAM |
| StreamStatus | 스트림 상태 | ACTIVE, INACTIVE, ERROR |
| UserRole | 사용자 역할 | ADMIN, KIOSK |
| NotificationStatus | 알림 상태 | PENDING, SENT, FAILED |

## 로봇 관련 모델

### Robot

기본 로봇 모델입니다. 다양한 종류의 로봇에 대한 공통 정보를 관리합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| robot_id | int | 로봇 식별자 |
| type | EntityType | 로봇 유형 |
| mac_address | str | MAC 주소 |
| ip_address | str | IP 주소 |
| timestamp | datetime | 타임스탬프 |

관계:
- orders: Order 모델과 1:N 관계
- commands: RobotCommand 모델과 1:N 관계
- cleaning_tasks: CleaningTask 모델과 1:N 관계

### Cookbot

조리를 담당하는 로봇 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| robot_id | int | 로봇 식별자 |
| status | RobotStatus | 로봇 상태 |
| timestamp | datetime | 타임스탬프 |

### Albabot

서빙을 담당하는 로봇 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| robot_id | int | 로봇 식별자 |
| status | RobotStatus | 로봇 상태 |
| battery_level | float | 배터리 수준 |
| timestamp | datetime | 타임스탬프 |

### RobotCommand

로봇에게 전송되는 명령을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| robot_id | int | 로봇 ID (외래 키) |
| command | str | 명령어 |
| parameters | Dict | 매개변수 (JSON) |
| status | CommandStatus | 명령 상태 |
| timestamp | datetime | 타임스탬프 |
| executed_at | datetime | 실행 시간 |

관계:
- robot: Robot 모델과 N:1 관계

### Pose6D

로봇의 6D 포즈(위치와 방향)를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| entity_id | int | 엔티티 ID |
| entity_type | EntityType | 엔티티 유형 |
| timestamp | datetime | 타임스탬프 |
| x | float | X 좌표 |
| y | float | Y 좌표 |
| z | float | Z 좌표 |
| roll | float | Roll 각도 |
| pitch | float | Pitch 각도 |
| yaw | float | Yaw 각도 |

### JointAngle

로봇 관절 각도를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| robot_id | int | 로봇 ID |
| timestamp | datetime | 타임스탬프 |
| joint_1 | float | 관절 1 각도 |
| joint_2 | float | 관절 2 각도 |
| joint_3 | float | 관절 3 각도 |
| joint_4 | float | 관절 4 각도 |
| joint_5 | float | 관절 5 각도 |
| joint_6 | float | 관절 6 각도 |

## 주문 관련 모델

### Order

고객 주문을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| customer_id | int | 고객 ID (외래 키) |
| robot_id | int | 로봇 ID (외래 키) |
| table_id | int | 테이블 ID |
| status | OrderStatus | 주문 상태 |
| timestamp | datetime | 타임스탬프 |
| served_at | datetime | 서빙 시간 |

관계:
- customer: Customer 모델과 N:1 관계
- robot: Robot 모델과 N:1 관계

### OrderItem

주문 항목을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| order_id | int | 주문 ID (외래 키, 복합 기본 키) |
| menu_item_id | int | 메뉴 항목 ID (외래 키, 복합 기본 키) |
| quantity | int | 수량 |
| status | OrderStatus | 주문 항목 상태 |

관계:
- menu_item: MenuItem 모델과 N:1 관계

### KioskTerminal

키오스크 단말기를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| table_id | int | 테이블 ID |
| ip_address | str | IP 주소 |

## 재고 관련 모델

### Inventory

재고 항목을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| ingredient_id | int | 재료 ID |
| name | str | 재료 이름 (외래 키) |
| count | int | 현재 수량 |
| max_count | int | 최대 수량 |
| status | InventoryStatus | 재고 상태 |
| updated_at | datetime | 업데이트 시간 |

### MenuItem

메뉴 항목을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| name | str | 메뉴 이름 |
| price | float | 가격 |
| prepare_time | int | 준비 시간 |
| image_url | str | 이미지 URL |
| description | str | 설명 |

관계:
- menu_ingredients: MenuIngredient 모델과 1:N 관계
- order_items: OrderItem 모델과 1:N 관계

### MenuIngredient

메뉴와 재료의 관계를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| name | str | 재료 이름 (고유) |
| menu_item_id | int | 메뉴 항목 ID (외래 키) |
| quantity_required | int | 필요 수량 |

관계:
- menu_item: MenuItem 모델과 N:1 관계

## 테이블 관련 모델

### Table

테이블 정보를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| max_customer | int | 최대 고객 수 |
| status | TableStatus | 테이블 상태 |
| updated_at | datetime | 업데이트 시간 |
| x | float | X 좌표 |
| y | float | Y 좌표 |
| width | float | 너비 |
| height | float | 높이 |

관계:
- assignments: GroupAssignment 모델과 1:N 관계

### GroupAssignment

테이블 배정 이력을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| table_id | int | 테이블 ID (외래 키) |
| customer_id | int | 고객 ID (외래 키) |
| timestamp | datetime | 타임스탬프 |
| released_at | datetime | 해제 시간 |

관계:
- table: Table 모델과 N:1 관계
- customer: Customer 모델과 N:1 관계

## 고객 관련 모델

### Customer

고객 그룹 정보를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| count | int | 인원 수 |
| timestamp | datetime | 타임스탬프 |

관계:
- assignments: GroupAssignment 모델과 1:N 관계
- orders: Order 모델과 1:N 관계

### FaceRecognition

얼굴 인식 정보를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| table_id | int | 테이블 ID |
| timestamp | datetime | 타임스탬프 |
| history | str | 히스토리 (JSON 문자열) |
| nowdetected | int | 현재 감지된 수 |
| reliability | int | 신뢰도 (0-100) |
| exist | int | 존재 여부 (0: 없음, 1: 있음) |

## 사용자 관련 모델

### User

시스템 사용자 정보를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| username | str | 사용자 이름 (고유) |
| password_hash | str | 비밀번호 해시 |
| name | str | 이름 |
| role | UserRole | 역할 |
| created_at | datetime | 생성 시간 |
| updated_at | datetime | 업데이트 시간 |
| last_login | datetime | 마지막 로그인 시간 |

관계:
- notifications: Notification 모델과 1:N 관계

### Notification

사용자 알림을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| user_id | int | 사용자 ID (외래 키) |
| type | str | 알림 유형 |
| message | str | 알림 메시지 |
| created_at | datetime | 생성 시간 |
| status | NotificationStatus | 알림 상태 |

관계:
- user: User 모델과 N:1 관계

## 시스템 관련 모델

### AdminSettings

관리자 설정을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| store_name | str | 매장 이름 |
| operation_start | str | 운영 시작 시간 |
| operation_end | str | 운영 종료 시간 |
| inventory_threshold | int | 재고 임계값 |
| alert_settings | Dict | 알림 설정 (JSON) |

### Event

시스템 이벤트를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| type | EventType | 이벤트 유형 |
| related_entity_type | EntityType | 관련 엔티티 유형 |
| related_entity_id | str | 관련 엔티티 ID |
| description | str | 설명 |
| timestamp | datetime | 타임스탬프 |

### SystemLog

시스템 로그를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| level | LogLevel | 로그 레벨 |
| message | str | 로그 메시지 |
| timestamp | datetime | 타임스탬프 |

### Emergency

비상 상황을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| emergency_type | str | 비상 유형 |
| description | str | 설명 |
| is_active | bool | 활성화 여부 |
| timestamp | datetime | 타임스탬프 |
| resolved_at | datetime | 해결 시간 |

## 미디어 관련 모델

### VideoStream

비디오 스트림을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| source_type | StreamSourceType | 소스 유형 |
| source_id | str | 소스 ID |
| last_checked | datetime | 마지막 확인 시간 |
| recording_started_at | datetime | 녹화 시작 시간 |
| recording_ended_at | datetime | 녹화 종료 시간 |
| url | str | 스트림 URL |
| status | StreamStatus | 스트림 상태 |
| recording_path | str | 녹화 경로 |

### Chat

채팅 메시지를 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| question | str | 질문 |
| answer | str | 답변 |
| timestamp | datetime | 타임스탬프 |
| robot_id | int | 로봇 ID (외래 키) |
| robot_task | str | 로봇 작업 |
| status | ChatStatus | 채팅 상태 |
| table_id | int | 테이블 ID |

## 청소 관련 모델

### CleaningTask

청소 작업을 관리하는 모델입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| id | int | 기본 키 |
| robot_id | int | 로봇 ID (외래 키) |
| area | str | 청소 영역 |
| status | str | 상태 |
| started_at | datetime | 시작 시간 |
| completed_at | datetime | 완료 시간 |

관계:
- robot: Robot 모델과 N:1 관계

## 모델 간 관계도

```
                                      ┌───────────┐
                                      │   Robot   │
                                      └─────┬─────┘
                                            │
                 ┌──────────────────────────┼──────────────────────────┐
                 │                          │                          │
         ┌───────▼────────┐       ┌─────────▼────────┐       ┌─────────▼────────┐
         │   Cookbot      │       │   RobotCommand   │       │   Albabot       │
         └────────────────┘       └──────────────────┘       └──────────────────┘
                                           │
                                  ┌────────┴─────────┐
                                  │                  │
                          ┌───────▼───────┐  ┌───────▼───────┐
                          │  CleaningTask │  │  Chat         │
                          └───────────────┘  └───────────────┘

┌───────────┐     ┌───────────┐     ┌───────────┐
│  Customer │◄────┤    Order  │◄────┤ OrderItem │
└─────┬─────┘     └─────┬─────┘     └─────┬─────┘
      │                 │                 │
      │                 │                 │
┌─────▼─────┐     ┌─────▼─────┐     ┌─────▼─────┐
│GroupAssign│     │   Robot   │     │ MenuItem  │
└─────┬─────┘     └───────────┘     └─────┬─────┘
      │                                   │
      │                                   │
┌─────▼─────┐                       ┌─────▼─────┐
│   Table   │                       │MenuIngred.│
└───────────┘                       └─────┬─────┘
                                          │
                                    ┌─────▼─────┐
                                    │ Inventory │
                                    └───────────┘

┌───────────┐     ┌───────────┐     ┌───────────┐
│    User   │◄────┤Notification│     │AdminSetting│
└───────────┘     └───────────┘     └───────────┘

┌───────────┐     ┌───────────┐     ┌───────────┐
│   Event   │     │ SystemLog │     │ Emergency │
└───────────┘     └───────────┘     └───────────┘

┌───────────┐     ┌───────────┐     ┌───────────┐
│VideoStream│     │FaceRecog. │     │   Pose6D  │
└───────────┘     └───────────┘     └───────────┘

┌───────────┐
│JointAngle │
└───────────┘
```

## 데이터 흐름

1. **주문 처리 흐름**:
   - Customer → Order → OrderItem → MenuItem → MenuIngredient → Inventory
   - Order → Robot → RobotCommand

2. **테이블 관리 흐름**:
   - Customer → GroupAssignment → Table
   - Table → FaceRecognition

3. **로봇 제어 흐름**:
   - Robot → RobotCommand
   - Robot → Pose6D, JointAngle
   - Robot → CleaningTask

4. **이벤트 및 알림 흐름**:
   - Event → User → Notification
   - Emergency → Event

5. **비디오 스트림 흐름**:
   - VideoStream → FaceRecognition 