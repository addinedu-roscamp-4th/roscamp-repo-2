# 로보다인 네트워크 통신 아키텍처

## 📌 개요

로보다인 서비스는 다양한 통신 프로토콜을 활용하여 로봇, 클라이언트, 외부 시스템 간의 효율적인 데이터 교환을 구현하고 있습니다. 이 문서는 로보다인 백엔드가 지원하는 다양한 네트워크 통신 방식과 그 구현 원리를 설명합니다.

## 🌐 지원하는 통신 프로토콜

로보다인 백엔드는 다음과 같은 통신 프로토콜을 지원합니다:

1. **HTTP/HTTPS REST API** - 기본적인 데이터 CRUD 작업
2. **WebSocket** - 실시간 양방향 통신
3. **TCP 소켓** - 로봇 상태 데이터 수신
4. **UDP** - 영상 스트림 및 짧은 제어 명령
5. **RTSP** - 실시간 비디오 스트리밍

## 📡 HTTP/HTTPS REST API

### 구현 방식

FastAPI를 사용하여 RESTful API를 구현하며, 주요 특징은 다음과 같습니다:

- 표준 HTTP 메서드(GET, POST, PUT, DELETE 등) 사용
- JSON 형식의 요청 및 응답 데이터
- 경로 파라미터 및 쿼리 파라미터를 통한 데이터 필터링
- Pydantic 모델을 사용한 데이터 검증
- JWT 토큰 기반 인증

```python
@router.get("/robots", response_model=List[RobotResponse])
async def get_robots(session: Session = Depends(get_session)):
    """모든 로봇 정보 조회"""
    robots = session.exec(select(Robot)).all()
    return robots
```

## 🔄 TCP 소켓 통신

### TCP 서버

로봇과의 주요 통신 채널로 TCP 소켓을 사용합니다. 로봇의 상태 업데이트, 명령 전송 등이 이루어집니다.

```python
class RoboDineTCPHandler(socketserver.BaseRequestHandler):
    def handle(self):
        raw = self.request.recv(1024).strip()
        try:
            payload = json.loads(raw.decode('utf-8'))
            
            with get_session() as session:
                # 페이로드 처리 및 데이터베이스 업데이트
                result = dispatch_payload(session, payload)
                
                # 영향받은 엔티티 정보 추출
                if isinstance(result, dict) and 'affected_entity' in result:
                    entity_type = result['affected_entity'].get('type')
                    entity_id = result['affected_entity'].get('id')
                
                session.commit()
                
                # 변경사항 웹소켓으로 브로드캐스팅
                if entity_type:
                    if main_loop:
                        main_loop.call_soon_threadsafe(
                            lambda: asyncio.create_task(broadcast_entity_update(entity_type, entity_id))
                        )
            
            response = "OK"
        except Exception as e:
            response = "ERROR"
        self.request.sendall(response.encode('utf-8'))
```

### TCP 서버 시작

```python
def start_tcp_server(host: str, port: int):
    server = socketserver.ThreadingTCPServer((host, port), RoboDineTCPHandler)
    server.serve_forever()

# 별도 스레드에서 TCP 서버 시작
tcp_thread = threading.Thread(target=start_tcp_server, args=("0.0.0.0", 8001), daemon=True)
tcp_thread.start()
```

## 📺 UDP 통신

UDP는 실시간성이 중요하고 일부 패킷 손실이 허용되는 영상 스트림 전송이나 빠른 제어 명령에 활용됩니다.

### UDP 수신기

`UDP_receiver.py` 파일은 UDP 프로토콜을 통한 비디오 프레임 수신 및 처리를 담당합니다:

```python
def main():
    # UDP 소켓 설정
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', args.port))
    sock.settimeout(1.0)

    while True:
        try:
            packet, addr = sock.recvfrom(65536)
            
            # 헤더에서 타임스탬프 추출
            ts_sent = struct.unpack('d', packet[:8])[0]
            jpeg_data = packet[8:]
            
            # 레이턴시 계산
            ts_recv = time.time()
            latency_ms = (ts_recv - ts_sent) * 1000.0
            
            # JPEG → BGR
            npdata = np.frombuffer(jpeg_data, dtype=np.uint8)
            frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)
            
            # 화면 표시 또는 저장
            cv2.imshow('UDP Video', frame)
            
            # 녹화 처리
            if recording and writer:
                writer.write(frame)
        except socket.timeout:
            continue
```

### UDP의 주요 특징

- 연결 설정 없음 (비연결성)
- 패킷 전송 순서 보장 없음
- 전송 신뢰성 보장 없음 (패킷 손실 가능)
- 오버헤드가 적어 빠른 전송 속도
- 실시간 화상/음성 전송에 적합

## 🎥 RTSP 스트리밍

RTSP(Real-Time Streaming Protocol)는 실시간 비디오 스트림 제공에 사용됩니다.

### RTSP 서버 설정

```python
# RTSP 서버 설정 및 시작
VIDEOS_DIR = "/home/addinedu/dev_ws/roscamp-repo-2/robodine_service/backend/videos"
os.makedirs(VIDEOS_DIR, exist_ok=True)

# rtsp-simple-server 사용
# app.mount("/videos", StaticFiles(directory=VIDEOS_DIR), name="videos")
```

### RTSP 스트림 관리

`video_streams.py` 라우터는 RTSP 스트림 관리 기능을 제공합니다:

- 스트림 등록 및 해제
- 스트림 상태 모니터링
- 녹화 시작 및 중지
- 스트림 URL 관리

## 📊 데이터 직렬화 및 변환

서로 다른 통신 채널 간의 데이터 교환을 위해 다양한 직렬화 함수를 구현:

```python
def serialize_robot(robot):
    """Robot 모델을 JSON 직렬화 가능한 형태로 변환"""
    robot_type = str(robot.type) if robot.type else None
    if robot_type:
        robot_type = robot_type.replace('EntityType.', '')
    return {
        "Robot.id": robot.id,
        "Robot.robot_id": robot.robot_id,
        "Robot.type": robot_type,
        "Robot.mac_address": robot.mac_address,
        "Robot.ip_address": robot.ip_address,
        "Robot.timestamp": robot.timestamp.isoformat() if robot.timestamp else None
    }
```

## 🔄 통신 흐름 및 연동

### 1. 로봇 → 백엔드 데이터 흐름

1. 로봇이 TCP 채널을 통해 상태 데이터 전송
2. `RoboDineTCPHandler`가 데이터 수신 및 처리
3. 데이터베이스에 상태 정보 저장
4. 웹소켓을 통해 프론트엔드에 실시간 업데이트

### 2. 로봇 → 백엔드 영상 전송

1. 로봇이 UDP 또는 RTSP를 통해 영상 스트림 전송
2. UDP 수신기 또는 RTSP 서버가 영상 처리
3. 필요시 영상 저장 또는 추가 처리(AI 분석 등)
4. 웹 클라이언트에 스트림 URL 제공

### 3. 백엔드 → 로봇 명령 전송

1. 관리자가 웹 인터페이스를 통해 명령 전송
2. REST API를 통해 백엔드 서버가 명령 수신
3. TCP 채널을 통해 로봇에 명령 전달
4. 로봇으로부터 명령 실행 결과 수신 및 피드백

## 🛡️ 보안 고려사항

### 1. 인증 및 권한 부여

- JWT 토큰 기반 API 인증
- 웹소켓 연결에 대한 최대 연결 수 제한
- IP 기반 필터링(선택적)

### 2. 데이터 암호화

- HTTPS를 통한 API 통신 암호화
- 중요 데이터 전송 시 추가 암호화 고려

### 3. 오류 처리 및 복원력

- 연결 실패 시 자동 재연결 메커니즘
- 폴백 폴링으로 데이터 일관성 유지
- 로깅을 통한 문제 진단

## 🔍 성능 모니터링

### 지연 시간 측정

UDP 스트림에서는 송신 및 수신 타임스탬프를 비교하여 지연 시간을 측정합니다:

```python
# 송신 timestmap 언패킹
ts_sent = struct.unpack('d', packet[:8])[0]

# 레이턴시 계산
ts_recv = time.time()
latency_ms = (ts_recv - ts_sent) * 1000.0
latencies.append(latency_ms)
avg_latency = sum(latencies) / len(latencies)
```

## 📝 결론

로보다인 백엔드는 다양한 통신 프로토콜을 활용하여 로봇 제어, 상태 모니터링, 영상 스트리밍 등의 기능을 효율적으로 구현하고 있습니다. HTTP/REST API, WebSocket, TCP, UDP, RTSP 등 각 통신 방식의 장점을 활용하여 다양한 요구사항에 최적화된 통신 아키텍처를 구축했습니다. 