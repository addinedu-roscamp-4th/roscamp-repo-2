# 📋 RoboDine 오류 코드 참조

## 📌 개요

이 문서는 RoboDine 서비스에서 발생할 수 있는 오류 코드와 그 의미, 그리고 해결 방법을 제공합니다. 오류 코드는 HTTP 상태 코드와 함께 세부적인 내부 코드로 구성되어 보다 정확한 문제 진단을 가능하게 합니다.

## 🚦 오류 응답 형식

RoboDine API에서 오류가 발생하면 다음과 같은 형식의 JSON 응답이 반환됩니다:

```json
{
  "detail": "오류에 대한 상세 메시지",
  "code": "ERROR_CODE",
  "status_code": 400
}
```

## 🔍 오류 코드 분류

### 1. 인증 및 권한 오류 (401, 403)

| 오류 코드 | HTTP 상태 코드 | 설명 | 해결 방법 |
|---------|--------------|-----|---------|
| `AUTHENTICATION_REQUIRED` | 401 | 인증이 필요한 리소스에 인증 없이 접근 | 유효한 액세스 토큰 사용 |
| `INVALID_CREDENTIALS` | 401 | 잘못된 인증 정보(사용자명/비밀번호) | 올바른 인증 정보 확인 |
| `TOKEN_EXPIRED` | 401 | 액세스 토큰이 만료됨 | 토큰 갱신 API를 통해 새 토큰 획득 |
| `TOKEN_INVALID` | 401 | 유효하지 않은 토큰 형식/서명 | 올바른 토큰 사용 |
| `TOKEN_REVOKED` | 401 | 취소된 토큰 | 다시 로그인하여 새 토큰 획득 |
| `PERMISSION_DENIED` | 403 | 요청된 작업에 대한 권한 없음 | 필요한 권한을 가진 계정 사용 |
| `ACCOUNT_DISABLED` | 403 | 비활성화된 계정 | 관리자에게 계정 활성화 요청 |

### 2. 리소스 오류 (404, 409)

| 오류 코드 | HTTP 상태 코드 | 설명 | 해결 방법 |
|---------|--------------|-----|---------|
| `RESOURCE_NOT_FOUND` | 404 | 요청한 리소스가 존재하지 않음 | 리소스 ID 확인 |
| `ROBOT_NOT_FOUND` | 404 | 지정된 로봇이 존재하지 않음 | 유효한 로봇 ID 사용 |
| `ORDER_NOT_FOUND` | 404 | 지정된 주문이 존재하지 않음 | 유효한 주문 ID 사용 |
| `TABLE_NOT_FOUND` | 404 | 지정된 테이블이 존재하지 않음 | 유효한 테이블 번호 사용 |
| `MENU_ITEM_NOT_FOUND` | 404 | 지정된 메뉴 항목이 존재하지 않음 | 유효한 메뉴 ID 사용 |
| `USER_NOT_FOUND` | 404 | 지정된 사용자가 존재하지 않음 | 유효한 사용자명 확인 |
| `RESOURCE_CONFLICT` | 409 | 리소스 충돌 (중복 등) | 충돌 조건 해결 |
| `ROBOT_UNAVAILABLE` | 409 | 로봇이 현재 사용 불가능한 상태 | 로봇 상태 확인 또는 다른 로봇 사용 |
| `TABLE_OCCUPIED` | 409 | 테이블이 이미 사용중 | 다른 테이블 선택 또는 대기 |
| `DUPLICATE_ORDER` | 409 | 중복된 주문 | 기존 주문 확인 |

### 3. 데이터 검증 오류 (422)

| 오류 코드 | HTTP 상태 코드 | 설명 | 해결 방법 |
|---------|--------------|-----|---------|
| `VALIDATION_ERROR` | 422 | 요청 데이터 유효성 검증 실패 | 오류 메시지에 따라 요청 데이터 수정 |
| `INVALID_FIELD_FORMAT` | 422 | 필드 형식이 유효하지 않음 | 올바른 형식 사용 |
| `REQUIRED_FIELD_MISSING` | 422 | 필수 필드 누락 | 모든 필수 필드 포함 |
| `VALUE_OUT_OF_RANGE` | 422 | 값이 유효 범위를 벗어남 | 유효 범위 내 값 사용 |
| `INVALID_STATUS_TRANSITION` | 422 | 유효하지 않은 상태 전환 | 현재 상태에서 가능한 전환 상태 확인 |
| `INVALID_ROBOT_COMMAND` | 422 | 로봇 유형에 맞지 않는 명령 | 로봇 유형에 맞는 명령 사용 |
| `INVALID_PAYMENT_INFO` | 422 | 결제 정보가 유효하지 않음 | 결제 정보 확인 |

### 4. 비즈니스 로직 오류 (400)

| 오류 코드 | HTTP 상태 코드 | 설명 | 해결 방법 |
|---------|--------------|-----|---------|
| `INSUFFICIENT_INVENTORY` | 400 | 재고 부족 | 재고 보충 또는 주문 변경 |
| `MENU_ITEM_UNAVAILABLE` | 400 | 주문한 메뉴 항목이 판매 중지됨 | 다른 메뉴 선택 |
| `ROBOT_BATTERY_LOW` | 400 | 로봇 배터리 부족 | 충전 후 재시도 |
| `COOKING_TIME_EXCEEDED` | 400 | 조리 시간 초과 | 관리자에게 문의 |
| `PAYMENT_FAILED` | 400 | 결제 처리 실패 | 결제 정보 확인 또는 다른 결제 방법 사용 |
| `ORDER_ALREADY_COMPLETED` | 400 | 이미 완료된 주문 수정 시도 | 완료된 주문은 수정 불가 |
| `ORDER_ALREADY_CANCELLED` | 400 | 이미 취소된 주문 처리 시도 | 취소된 주문은 처리 불가 |

### 5. 시스템 오류 (500)

| 오류 코드 | HTTP 상태 코드 | 설명 | 해결 방법 |
|---------|--------------|-----|---------|
| `SYSTEM_ERROR` | 500 | 내부 서버 오류 | 개발팀에 문의 |
| `DATABASE_ERROR` | 500 | 데이터베이스 작업 실패 | 개발팀에 문의 |
| `ROBOT_COMMUNICATION_ERROR` | 500 | 로봇과의 통신 실패 | 네트워크 연결 확인 |
| `THIRD_PARTY_SERVICE_ERROR` | 500 | 외부 서비스 연동 오류 | 외부 서비스 상태 확인 |
| `FILE_SYSTEM_ERROR` | 500 | 파일 시스템 작업 오류 | 디스크 공간 및 권한 확인 |
| `UNEXPECTED_ERROR` | 500 | 예상치 못한 오류 | 개발팀에 문의 및 로그 확인 |

### 6. 네트워크 및 통신 오류

| 오류 코드 | HTTP 상태 코드 | 설명 | 해결 방법 |
|---------|--------------|-----|---------|
| `CONNECTION_TIMEOUT` | 504 | 연결 시간 초과 | 네트워크 상태 확인 또는 재시도 |
| `CONNECTION_REFUSED` | 503 | 연결 거부 | 대상 서비스 실행 여부 확인 |
| `WEBSOCKET_DISCONNECTED` | - | WebSocket 연결 끊김 | 자동 재연결 기다리거나 페이지 새로고침 |
| `WEBSOCKET_CONNECTION_LIMIT` | - | WebSocket 연결 한도 초과 | 불필요한 연결 종료 후 재시도 |
| `TCP_SOCKET_ERROR` | - | TCP 소켓 오류 | 네트워크 연결 및 포트 확인 |
| `UDP_PACKET_LOSS` | - | UDP 패킷 손실 | 네트워크 상태 개선 또는 중요 데이터는 TCP 사용 |

## 🔄 상태 코드와 트랜지션 오류

### 로봇 상태 전환 오류

| 현재 상태 | 요청된 상태 | 오류 코드 | 설명 |
|---------|-----------|---------|------|
| `ERROR` | `BUSY` | `INVALID_ROBOT_TRANSITION` | 오류 상태에서 직접 작업 상태로 전환 불가 |
| `CHARGING` | `IDLE` | `INSUFFICIENT_BATTERY` | 충분한 배터리 충전 전 대기 상태로 전환 불가 |
| `MAINTENANCE` | `BUSY` | `ROBOT_IN_MAINTENANCE` | 유지보수 중인 로봇은 작업 수행 불가 |

### 주문 상태 전환 오류

| 현재 상태 | 요청된 상태 | 오류 코드 | 설명 |
|---------|-----------|---------|------|
| `COMPLETED` | `COOKING` | `INVALID_ORDER_TRANSITION` | 완료된 주문을 조리 중으로 변경 불가 |
| `CANCELLED` | `SERVING` | `INVALID_ORDER_TRANSITION` | 취소된 주문을 서빙 중으로 변경 불가 |
| `PENDING` | `COMPLETED` | `INVALID_ORDER_TRANSITION` | 대기 중 주문을 직접 완료로 변경 불가 |

## 📱 클라이언트 측 오류 처리

클라이언트는 오류를 다음과 같이 처리할 수 있습니다:

```javascript
async function fetchData(url) {
  try {
    const response = await fetch(url, {
      headers: {
        'Authorization': `Bearer ${accessToken}`
      }
    });
    
    if (!response.ok) {
      const errorData = await response.json();
      handleApiError(errorData);
      return null;
    }
    
    return await response.json();
  } catch (error) {
    console.error("Network error:", error);
    return null;
  }
}

function handleApiError(errorData) {
  switch(errorData.code) {
    case 'TOKEN_EXPIRED':
      refreshToken();
      break;
    case 'PERMISSION_DENIED':
      showPermissionError();
      break;
    case 'RESOURCE_NOT_FOUND':
      showNotFoundError();
      break;
    case 'VALIDATION_ERROR':
      showFieldErrors(errorData.detail);
      break;
    // ... 다른 오류 처리
    default:
      showGenericError();
  }
}
```

## 🧪 테스트 중 오류 시뮬레이션

개발 및 테스트 환경에서는 특정 API 엔드포인트로 오류를 강제로 발생시켜 클라이언트 오류 처리를 테스트할 수 있습니다:

```
GET /api/test/error/{error_code}
```

이 엔드포인트는 지정된 오류 코드에 해당하는 오류 응답을 반환합니다.

## 📝 오류 로깅 및 모니터링

모든 오류는 다음 정보와 함께 로깅됩니다:

- 오류 코드 및 메시지
- 요청 URL 및 메서드
- 요청 헤더 및 본문 (민감한 정보 제외)
- 사용자 ID (인증된 경우)
- 타임스탬프
- 스택 트레이스 (개발 환경)

로그는 ELK 스택(Elasticsearch, Logstash, Kibana)을 통해 모니터링됩니다. 