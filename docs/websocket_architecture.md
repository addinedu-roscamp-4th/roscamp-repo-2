# 로보다인 실시간 통신: 웹소켓 아키텍처

## 📌 웹소켓 통신 개요

로보다인 서비스는 실시간 데이터 업데이트를 위해 웹소켓 기반의 양방향 통신 시스템을 구현하고 있습니다. 이 문서는 웹소켓 통신 아키텍처의 구조와 작동 원리에 대해 설명합니다.

## 🏗️ 웹소켓 핵심 구성요소

### 1. ConnectionManager

웹소켓 연결 관리를 담당하는 핵심 클래스로, 클라이언트 연결 관리 및 메시지 브로드캐스팅을 처리합니다.

```python
class ConnectionManager:
    def __init__(self):
        # 토픽별 활성 연결 저장
        self.active_connections: Dict[str, List[WebSocket]] = {
            "robots": [],
            "tables": [],
            "events": [],
            "orders": [],
            "status": [],
            "systemlogs": [],
            "customers": [],
            "inventory": [],
            "video_streams": []
        }
        self.shutting_down = False
        self.max_connections_per_topic = WS_MAX_CONNECTIONS
```

### 2. 토픽 기반 구독 시스템

클라이언트는 특정 토픽을 구독하여 관련 업데이트만 수신할 수 있습니다:

- `robots`: 로봇 상태 업데이트
- `tables`: 테이블 상태 업데이트
- `events`: 이벤트 발생 알림
- `orders`: 주문 상태 변경
- `status`: 전체 시스템 상태
- `systemlogs`: 시스템 로그 업데이트
- `customers`: 고객 정보 업데이트
- `inventory`: 재고 상태 업데이트
- `video_streams`: 비디오 스트림 정보

## 🔄 메시지 프로토콜

웹소켓을 통해 전송되는 메시지는 표준화된 JSON 형식을 따릅니다:

```python
class WSMessage(BaseModel):
    type: Literal["update", "error", "ping", "pong", "shutdown"]
    topic: Literal["robots", "tables", "events", "orders", "status", "systemlogs", "customers", "inventory", "video_streams"]
    data: Any
```

### 메시지 타입

- `update`: 데이터 업데이트 알림
- `error`: 오류 메시지
- `ping`/`pong`: 연결 유지 확인
- `shutdown`: 서버 종료 알림

## 🌐 웹소켓 엔드포인트

### 통합 웹소켓 엔드포인트

```python
@router.websocket("/ws/{topic}")
async def websocket_topic_endpoint(websocket: WebSocket, topic: str):
    """단일 통합 웹소켓 엔드포인트 - 토픽은 URL 경로 파라미터로 지정"""
    # 지원되는 토픽 확인
    valid_topics = ["robots", "tables", "events", "orders", "status", "systemlogs", "customers", "inventory", "video_streams"]
    if topic not in valid_topics:
        await websocket.close(code=1003)  # 1003 = Unsupported data
        return
        
    # 연결 처리
    connected = await manager.connect(websocket, topic)
    if not connected:
        return  # 연결 실패 시 즉시 반환
    
    # ... 연결 처리 및 메시지 루프
```

## 💡 주요 기능

### 1. 연결 관리

```python
async def connect(self, websocket: WebSocket, topic: str):
    """웹소켓 연결 수락 및 토픽에 등록"""
    # 종료 중이면 연결 거부
    if self.shutting_down:
        await websocket.close(code=1001, reason="Server is shutting down")
        return False
        
    # 토픽별 최대 연결 수 제한
    if len(self.active_connections.get(topic, [])) >= self.max_connections_per_topic:
        await websocket.close(code=1008, reason="Too many connections")
        return False
        
    await websocket.accept()
    if topic not in self.active_connections:
        self.active_connections[topic] = []
        
    self.active_connections[topic].append(websocket)
    return True
```

### 2. 연결 해제 처리

```python
def disconnect(self, websocket: WebSocket, topic: str):
    """웹소켓 연결 해제 및 토픽에서 제거"""
    if topic in self.active_connections and websocket in self.active_connections[topic]:
        self.active_connections[topic].remove(websocket)
```

### 3. 브로드캐스팅

```python
async def broadcast(self, message_data: dict, topic: str):
    """특정 토픽에 등록된 모든 클라이언트에 메시지 브로드캐스팅"""
    if topic not in self.active_connections:
        return
    
    message_str = json.dumps(message_data)
    failed_connections = []
    
    for connection in self.active_connections[topic]:
        try:
            await connection.send_text(message_str)
        except Exception as e:
            failed_connections.append(connection)
            
    # 실패한 연결은 나중에 한꺼번에 제거
    for connection in failed_connections:
        self.disconnect(connection, topic)
```

### 4. 핑-퐁 연결 유지

```python
async def send_ping_to_all(self):
    """모든 연결에 정기적으로 핑 메시지 전송"""
    while not self.shutting_down:
        try:
            ping_message = {
                "type": "ping",
                "topic": "status",
                "data": {"timestamp": asyncio.get_event_loop().time()}
            }
            ping_str = json.dumps(ping_message)
            
            for topic, connections in self.active_connections.items():
                for conn in connections[:]:  # 복사본으로 반복
                    try:
                        await conn.send_text(ping_str)
                    except Exception as e:
                        self.disconnect(conn, topic)
                        
            # 핑 간격만큼 대기
            await asyncio.sleep(WS_PING_INTERVAL)
        except asyncio.CancelledError:
            break
```

## 🔄 데이터 흐름

### 1. 초기 연결 및 데이터 로드

1. 클라이언트가 `/ws/{topic}` 엔드포인트에 연결
2. 서버는 연결을 수락하고 토픽별 구독 목록에 추가
3. 특정 토픽(예: robots, inventory)에 연결 시 초기 데이터를 즉시 전송

```python
# 초기 브로드캐스트: 로봇 연결 시 전체 상태 즉시 전송
if topic == "robots":
    try:
        await broadcast_entity_update("robot", None)
    except Exception as e:
        logger.error(f"Initial robot broadcast failed: {e}")
```

### 2. 실시간 업데이트 처리

1. 데이터베이스 변경 발생 시(TCP 핸들러를 통해 수신):
   ```python
   if entity_type:
       main_loop.call_soon_threadsafe(
           lambda: asyncio.create_task(broadcast_entity_update(entity_type, entity_id))
       )
   ```

2. 변경된 데이터를 데이터베이스에서 조회:
   ```python
   def _sync_db_work():
       with Session(engine) as session:
           # 데이터베이스 조회 작업
           # ...
   
   result = await anyio.to_thread.run_sync(_sync_db_work)
   ```

3. 해당 토픽 구독자에게 브로드캐스팅:
   ```python
   await broadcast_robots_update(result['data'])
   # 또는
   await broadcast_tables_update(combined)
   ```

### 3. 백업 폴링 메커니즘

데이터 업데이트가 실패하는 경우를 대비한 폴백 메커니즘:

```python
async def fallback_polling():
    """데이터베이스 폴링 백업 메커니즘 (이벤트 놓침 대비)"""
    while True:
        try:
            # 각 엔티티 타입에 대해 마지막 브로드캐스트 이후 일정 시간이 지났으면 갱신
            now = datetime.now()
            
            # 로봇 데이터 10초마다 갱신
            if (now - last_broadcast["robots"]).total_seconds() > POLLING_INTERVAL:
                await broadcast_entity_update("robot", None)
                
            # 다른 토픽도 유사하게 처리
            # ...
                
        except Exception as e:
            logger.error(f"Error in fallback polling: {str(e)}")
            
        await asyncio.sleep(POLLING_INTERVAL)
```

## 🔒 오류 처리 및 강건성

### 1. 연결 종료 처리

```python
async def shutdown(self):
    """모든 웹소켓 연결을 깔끔하게 닫는 함수"""
    self.shutting_down = True
    
    # 모든 클라이언트에 종료 메시지 전송
    shutdown_message = {
        "type": "shutdown",
        "topic": "status",
        "data": {"reason": "Server is shutting down"}
    }
    
    shutdown_str = json.dumps(shutdown_message)
    for topic, connections in self.active_connections.items():
        for conn in connections[:]:
            try:
                await conn.send_text(shutdown_str)
                await conn.close(code=1001)
            except Exception as e:
                pass
```

### 2. 브로드캐스팅 오류 처리

```python
failed_connections = []
for connection in self.active_connections[topic]:
    try:
        await connection.send_text(message_str)
    except Exception as e:
        failed_connections.append(connection)
        
# 실패한 연결은 나중에 한꺼번에 제거
for connection in failed_connections:
    self.disconnect(connection, topic)
```

## 📋 웹소켓 응용 시나리오

### 1. 로봇 상태 모니터링

1. 관리자 대시보드가 `status` 토픽 구독
2. 로봇 상태 변경 시 실시간으로 업데이트 수신
3. 대시보드 UI에 로봇 위치 및 상태 표시

### 2. 실시간 주문 관리

1. 주방 디스플레이가 `orders` 토픽 구독
2. 새 주문 발생 시 실시간 알림
3. 주문 상태 변경(조리 중, 완료 등) 표시

### 3. 시스템 모니터링

1. 유지보수 콘솔이 `systemlogs` 토픽 구독
2. 시스템 오류 및 경고 실시간 모니터링
3. 중요 이벤트 발생 시 즉시 대응

## 🚀 클라이언트 구현 가이드

### 브라우저 JavaScript 예제

```javascript
const connectWebSocket = (topic) => {
  const ws = new WebSocket(`ws://server:port/ws/${topic}`);
  
  ws.onopen = () => {
    console.log(`Connected to ${topic} updates`);
  };
  
  ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    
    // 메시지 타입에 따라 처리
    if (message.type === "update") {
      updateUI(message.data);
    } else if (message.type === "ping") {
      // 퐁 응답 보내기
      ws.send(JSON.stringify({
        type: "pong",
        topic: topic,
        data: { timestamp: message.data.timestamp }
      }));
    }
  };
  
  ws.onclose = () => {
    console.log(`Disconnected from ${topic}`);
    // 재연결 로직
    setTimeout(() => connectWebSocket(topic), 3000);
  };
  
  return ws;
};
```

## 📝 결론

로보다인의 웹소켓 아키텍처는 확장성과 신뢰성을 고려한 설계로, 토픽 기반 구독 모델을 통해 효율적인 실시간 통신을 제공합니다. 핑-퐁 메커니즘과 백업 폴링 시스템은 연결 안정성을 확보하며, 명확한 메시지 프로토콜은 클라이언트-서버 간 일관된 통신을 가능하게 합니다. 