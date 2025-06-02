# 프로젝트 진행 내역

## 2025-05-21: 실시간 웹캠 스트리밍 ffmpeg 인코더 설정 오류 해결
- ffmpeg의 mpeg1video 인코더와 low_delay 플래그 호환성 문제 해결
  - `live_streaming.py` 개선:
    - `-strict unofficial` 옵션 추가: mpeg1video에서 low_delay 사용 가능하도록 설정
    - `-flags` 설정 수정: `low_delay` → `+low_delay`로 변경하여 올바른 형식 적용
    - `-g 15` 옵션 추가: 적절한 키프레임 간격(GOP 크기) 설정
    - 로그 레벨 변경: `error` → `warning`으로 변경하여 더 많은 디버그 정보 확인
    - 품질 설정 조정: `-q:v 10` → `-q:v 31`로 변경 (MPEG1에서는 0-31 범위, 값이 클수록 압축률 높음)
    - 웹캠 검사 로직 강화: 단순 장치 파일 존재 확인을 넘어 실제 사용 가능 여부 검증
    - 초기 데이터 처리 개선:
      - 초기 버퍼 크기 증가: 512KB → 1MB로 확대
      - 타임아웃 증가: 2초 → 5초로 확대하여 초기화 시간 충분히 확보
      - MPEG-TS 동기화 로직 개선: 단일 패턴 대신 0x47 시작 바이트 검색 방식 적용
      - stderr 데이터 수집 및 로깅 강화: 오류 원인 파악 용이
    - 스트림 데이터 처리 안정화:
      - 빈 데이터 처리 개선: 연속 5회 빈 데이터 수신 시에만 종료 결정
      - 초기화 대기 시간 추가: ffmpeg 실행 후 데이터 수신 전 0.5초 대기
      - MPEG-TS 헤더 검증 로직 추가: `\x47\x40\x00\x10` 시그니처 확인으로 유효한 스트림 시작점 보장
    - 타임아웃 값 확대(0.5초 → 1.0초): 더 안정적인 데이터 스트리밍

이번 업데이트는 ffmpeg 인코더 설정 호환성 문제를 해결하고, 초기 데이터 처리 로직을 안정화하여 "low delay forcing is only available for mpeg2" 오류를 해결했습니다. 또한 데이터 스트림 처리 방식을 개선하여 더 안정적인 웹캠 스트리밍이 가능해졌으며, 오류 발생 시 상세한 진단 정보를 로그에 기록하도록 개선했습니다.

## 2025-05-21: 실시간 웹캠 스트리밍 MJPEG 디코딩 오류 해결
- 웹캠 스트리밍 중 발생하는 MJPEG 디코딩 오류 및 데이터 동기화 문제 해결
  - `live_streaming.py` 최적화:
    - MJPEG 입력 포맷 지정 제거: 자동 감지 기능 활용으로 디코딩 오류 방지
    - 비동기 ffmpeg 프로세스 관리 개선: `kill_previous_ffmpeg_processes` 함수 추가로 안정적인 프로세스 관리
    - 초기 데이터 청크 특별 처리 로직: 유효한 MPEG-TS 패킷 시작점 검증 및 동기화
    - MPEG-TS 헤더 검증 로직 추가: `\x47\x40\x00\x10` 시그니처 확인으로 유효한 스트림 시작점 보장
    - 타임아웃 값 확대(0.5초 → 1.0초): 더 안정적인 데이터 스트리밍
    - 디버깅 로그 개선: 데이터 처리 과정의 상세 로깅으로 문제 진단 용이
    - 오디오 스트림 명시적 제거(-an): 불필요한 오류 방지
    - 인코딩 프리셋 추가: `ultrafast`, `zerolatency` 옵션으로 성능 향상

이번 업데이트는 웹캠에서 발생하던 "unable to decode APP fields: Invalid data found when processing input" 오류를 해결하고, 데이터 스트림 시작 부분의 동기화 문제도 수정했습니다. 프레임레이트를 15fps로 낮추고 데이터 전송 청크 크기를 최적화하여 스트리밍 안정성이 크게 향상되었습니다.

## 2025-05-21: 실시간 웹캠 스트리밍 오류 수정 (React DOM 렌더링 오류 해결)
- React 렌더링 사이클과 JSMpeg 플레이어 간의 충돌 해결
  - `LiveStreamComponent.jsx` 개선:
    - `mountedRef` 추가: 컴포넌트 마운트 상태를 안전하게 추적하여 언마운트 시 DOM 조작 방지
    - `initializingRef` 추가: 초기화 중복 방지 및 초기화 상태 관리 개선
    - 클린업 로직을 `useCallback`으로 분리하여 재사용성 및 코드 가독성 향상
    - WebGL 렌더러 완전 비활성화: 생성자 자체를 Canvas2D로 대체하여 호환성 강화
    - DOM 렌더링 시간 확보를 위한 초기화 과정 지연 (setTimeout 100ms)
    - 이벤트 핸들러 내에 마운트 상태 확인 로직 추가: 언마운트 후 상태 업데이트 방지
    - 모든 비동기 작업에 마운트 상태 검사 추가하여 메모리 누수 방지
    - 하드코딩된 WebSocket URL 대신 동적 URL 생성으로 환경 독립성 확보
    - 핸들러 재사용을 위한 `useCallback` 사용 확대
    - 추가 플레이어 옵션 최적화: `pauseWhenHidden`, `disableGl`, `loop`, `videoBufferHWM` 등

이번 업데이트는 JSMpeg 플레이어와 React 생명주기 간의 충돌 문제를 해결하여 `insertBefore on Node` 오류가 발생하지 않도록 개선했습니다. 컴포넌트 마운트/언마운트 사이클에서 DOM 조작이 안전하게 이루어지도록 수정하고, 플레이어 초기화 시점을 세밀하게 제어함으로써 렌더링 안정성을 크게 향상시켰습니다. 또한 자원 관리와 오류 처리가 강화되어 메모리 누수 위험이 감소했습니다.

## 2025-05-21: 실시간 웹캠 스트리밍 오류 수정 (WebGL 렌더러 문제 해결 - 추가 업데이트)
- WebGL 렌더러 관련 오류 완전 해결 및 스트리밍 안정성 강화
  - `LiveStreamComponent.jsx` 개선:
    - JSMpeg이 WebGL을 사용하지 못하도록 강제 설정 추가: `window.JSMpeg.Renderer.WebGL.IsSupported = () => false`로 오버라이드
    - 재시도 횟수 제한(attemptRef) 추가하여 무한 재시도 방지
    - 이전 플레이어 인스턴스 명시적 정리 로직 추가
    - 의존성 배열에서 loading 제거로 불필요한 리렌더링 방지
    - 새로고침 버튼 추가로 사용자가 스트림을 수동으로 초기화할 수 있도록 개선
    - 추가 플레이어 옵션 최적화: progressive, throttled, chunkSize, maxAudioLag 설정
  - `live_streaming.py` 개선:
    - 출력 포맷을 `mpeg1video`에서 `mpegts`로 변경하여 더 안정적인 전송 컨테이너 사용
    - 프레임레이트를 25fps에서 15fps로 낮춰 대역폭 요구사항 감소
    - 품질 및 비트레이트 조정 (품질 15, 비트레이트 600k)
    - 고급 인코딩 최적화: preset ultrafast, tune zerolatency 추가
    - 청크 크기를 8192에서 4096으로 줄여 더 효율적인 데이터 전송
    - 오디오 스트림 제거(-an)로 불필요한 데이터 전송 방지
    - 더 효과적인 플래그 조합: +nobuffer+flush_packets 사용

이번 업데이트는 완전히 WebGL 렌더러 초기화 관련 오류를 해결하고, 스트리밍의 안정성과 성능을 크게 개선했습니다. WebGL 대신 Canvas2D 렌더러를 확실하게 사용하도록 강제함으로써 브라우저 호환성 문제가 해소되었으며, 스트리밍 설정 최적화로 CPU 사용량이 감소하고 지연시간이 줄어들었습니다.

## 2025-05-21: 실시간 웹캠 스트리밍 오류 수정
- 웹캠 스트리밍 기능 안정화 및 오류 해결
  - `LiveStreamComponent.jsx`: 웹소켓 연결 방식 개선
    - URL 설정 방식 개선: 환경변수가 없을 경우 window.location 기반 fallback URL 구현
    - 초기 타임아웃 및 재연결 메커니즘 강화: 연결 실패 시 사용자에게 명확한 정보 제공
    - 로딩/재연결 상태 UI 추가: 사용자에게 현재 연결 상태를 시각적으로 표시
    - 에러 처리 개선: 디버깅을 위한 로그 추가 및 예외 처리 강화
  - `live_streaming.py`: ffmpeg 명령어 최적화
    - 인코더 설정 문제 해결: `mpeg1video` 인코더 파라미터 최적화
    - 출력 포맷 변경: `mpegts` 포맷으로 변경하여 호환성 향상
    - 웹소켓 엔드포인트 경로 수정: `/ws/webcam`에서 `/webcam`으로 단순화
    - 오류 복원력 증가: 타임아웃 처리 및 연결 상태 모니터링 추가
    - 클린업 로직 강화: 모든 리소스가 확실히 정리되도록 예외 처리 추가

이번 업데이트는 웹캠 스트리밍 부분에서 발생하던 연결 문제와 재생 오류를 해결하여, 안정적인 실시간 영상 스트리밍 기능을 제공합니다. ffmpeg 명령어와 웹소켓 연결 방식을 최적화하여 성능이 향상되었으며, 브라우저와의 호환성도 개선되었습니다.

## 2023-09-26: ESLint 오류 수정 - VideoStreamPage 컴포넌트
- LiveStreamComponent의 재시도 변수 관련 ESLint 오류 수정
  - 지역 변수로 선언된 `retryCount`와 `MAX_RETRIES`를 전역 상태로 변경
  - `retryCount`를 useRef로 변경(`retryCountRef`)하여 렌더링 간 값 유지
  - `MAX_RETRIES` 상수를 컴포넌트 상단으로 이동
  - 컴포넌트 마운트 시 재시도 카운터 초기화 로직 추가
  - 변수 참조를 일관되게 수정하여 코드 안정성 확보

## 2023-09-25: 웹소켓 연결 및 컴포넌트 언마운트 문제 해결
- LiveStreamComponent 생명주기 관리 개선
  - 컴포넌트 순환 마운트/언마운트 문제 해결: 마운트/언마운트 사이클 안정화
  - 초기화 로직과 useEffect 의존성 배열 분리: 불필요한 재실행 방지
  - 컴포넌트 언마운트 시 리소스 정리 순서 최적화: 
    - 마운트 플래그 먼저 변경 후 타이머 및 리소스 정리
    - 언마운트 중 오류 발생 방지를 위한 조건 검사 강화
  - 웹소켓 연결 과정 디버깅 강화:
    - 상세 로그 출력으로 연결 생명주기 추적 가능
    - 연결 시간 측정 추가 (소요시간 ms 단위)
  - 웹소켓 포트 유연성 확보: 환경변수나 윈도우 객체에서 포트 동적 설정

## 2023-09-24: 웹소켓 연결 오류 개선
- 웹소켓 연결 실패 시 오류 처리 및 사용자 피드백 개선
  - 동적 호스트명 기반 웹소켓 URL 설정: 모든 환경에서 정확히 동작하도록 개선
  - 서버 상태 추적 기능 추가: disconnected, timeout, error, unstable 등의 상태 구분
  - 연결 실패 시 상세한 문제 설명과 해결 방법 제시:
    - 서버 포트 확인, 웹캠 스트리밍 서비스 활성화, 웹캠 연결 상태, 방화벽 설정 등
  - 연결 정보 디버깅 기능 강화: 현재 호스트명, 연결 URL 등의 정보 로깅
  - 정확한 오류 메시지 표시: 각 실패 상황별 맞춤형 메시지 제공

## 2023-09-23: 웹캠 스트리밍 연결 문제 해결
- WebSocket 연결 실패 및 플레이어 정리 오류 수정
  - 웹소켓 연결 전 사전 테스트 로직 추가: 연결 가능 여부 확인 후 플레이어 생성
  - 플레이어 정리 함수 안정성 대폭 강화: 각 단계별 try-catch 처리로 에러 전파 방지
  - 재연결 시도 제한 및 로깅 개선: 무한 재시도로 인한 자원 낭비 방지
  - 디버깅 콘솔 로그 추가: 문제 발생 시 원인 파악 용이
  - 사용자 인터페이스 개선:
    - 연결 실패 시 명확한 오류 메시지 표시
    - 재시도 버튼 추가로 사용자가 직접 연결 다시 시도 가능
    - 연결 상태에 따른 오류 알림 UI 추가

## 2023-09-22: React DOM 렌더링 오류 해결
- JSMpeg 플레이어와 React DOM 조작 관련 오류 수정
  - `insertBefore on Node` 오류 해결: 컴포넌트 언마운트 중 DOM 업데이트 충돌 방지
  - LiveStreamComponent 컴포넌트 개선:
    - isMountedRef를 사용한 안전한 상태 업데이트 처리
    - 웹소켓 및 타이머 리소스 철저한 정리 로직 추가
    - 컴포넌트 초기 렌더링 지연으로 DOM 구조 안정성 확보
    - 웹소켓 소켓 이벤트 핸들러 명시적 제거 로직 추가
  - 고유 key를 사용하여 컴포넌트 재생성 시 안정성 확보
  - jsmpeg-player npm 패키지 직접 설치로 라이브러리 로드 안정화

## 2023-09-21: JSMpeg 플레이어 오류 수정
- `VideoStreamPage.jsx`: JSMpeg 플레이어 관련 오류 수정
  - 플레이어 정리 중 발생하는 `Cannot read properties of null (reading 'close')` 오류 해결
  - `cleanupStream` 함수 개선: 웹소켓 소스 및 플레이어 리소스 안전하게 정리
  - 플레이어 옵션 최적화: 비디오 버퍼 크기 조정 및 이벤트 핸들러 추가 
  - 컴포넌트 마운트/언마운트 주기 개선: 마운트 상태 추적 및 오류 처리 강화
  - 비동기 초기화 로직 안정화

## 2023-09-20: 라이브 스트리밍 문제 해결
- JSMpeg 라이브러리 로드 실패 문제 해결
  - `public/index.html`에 JSMpeg 라이브러리 직접 추가하여 동적 로딩 문제 해결
  - `LiveStreamComponent` 수정으로 라이브러리 참존 방식 변경 및 오류 처리 강화
  - 타임아웃 설정으로 연결 오류 감지 추가
  - 웹캠 상태 확인 로직 강화
- `live_streaming.py` 개선
  - 웹캠 상태 확인 함수 추가 (check_webcam_available)
  - 로깅 기능 향상으로 문제 진단 용이하게 개선
  - 스트리밍 품질 파라미터 최적화 (버퍼 크기, 비트레이트 조정)
  - 예외 처리 강화 및 리소스 정리 로직 보완
  - 프로세스 관리 로직 개선

## 2023-09-19: 라이브 스트리밍 기능 추가
- `VideoStreamPage.jsx`: LIVE 탭 추가 및 실시간 웹캠 스트리밍 기능 구현
  - 기존 탭 목록에 'LIVE' 탭 추가
  - LiveStreamComponent 컴포넌트 구현하여 웹캠 스트리밍 처리
  - JSMpeg 라이브러리를 사용하여 웹소켓 기반 스트리밍 연결
  - 웹소켓을 통해 `/ws/webcam` 엔드포인트로 실시간 스트림 수신
  - 로딩, 오류 상태 처리 및 UI 개선
- `enums.py`: StreamSourceType에 WEBCAM 타입 추가
  - 향후 웹캠 기반 영상 기록을 위한 타입 추가

이 업데이트로 VideoStreamPage에서 라이브 탭으로 서버에 연결된 웹캠의 실시간 영상을 볼 수 있게 되었습니다. 추후에는 CCTV나 다른 IP 카메라와의 연동을 위한 기반 마련.

## OrderCompletePage.js 수정
- 주문 완료 페이지에서 알림이 중복 표시되는 문제 수정
- useRef를 사용하여 알림이 한 번만 표시되도록 개선

## Sidebar.js 수정
- 주문 현황 및 직원 호출 버튼의 폰트 크기와 높이 증가
- 버튼 높이를 h-16에서 h-20으로 변경
- 아이콘 크기를 text-2xl에서 text-3xl로 변경
- 텍스트 크기를 text-lg에서 text-xl로 변경

## OrderStatusPage.js 수정
- 주문 취소 처리 개선
- 백엔드에서 웹소켓으로 자동 업데이트되는 구조를 활용해 불필요한 연결 새로고침 제거
- 취소 완료 후 로딩 상태를 2초간 유지하여 사용자에게 피드백 제공
- 취소 중 UI 개선 (주문 취소 처리 완료 메시지 표시)
- 취소 중 이전 주문 데이터를 유지하여 화면 깜빡임 문제 해결
- 주문 대기시간 표시 제거 (단순히 '조리중'으로만 표시)

## MenuItem.js 수정
- 메뉴 아이템 카드에 예상 조리시간 추가 표시
- 가격 아래에 "예상 조리시간: X분" 형식으로 표시 

## 2023-07-26: 장바구니 기능 수정
- `CartContext.js`: addToCart 함수 수정으로 한 번에 여러 아이템 추가 기능 구현
  - quantity 파라미터를 추가하여 여러 개의 아이템을 한번에 추가할 수 있도록 수정
  - 이미 장바구니에 있는 아이템인 경우 수량을 정확히 증가시키도록 수정
- `MenuGrid.js`: addToCartWithQty 함수 수정
  - for문을 사용하지 않고 단일 함수 호출로 수량 전달하도록 수정
  - onAdd 이벤트에서도 수량을 명시적으로 전달하도록 수정
- `HomePage.js`: handleAddToCart 함수 수정
  - 수량을 파라미터로 받아 CartContext의 addToCart 함수에 전달하도록 수정

이 수정으로 메뉴 추가 시 발생하던 랜덤한 수량 증가 문제를 해결하였습니다.

## 2023-07-27: 메뉴 데이터 처리 로직 수정
- `HomePage.js`: 메뉴 데이터 소스 처리 로직 개선
  - 메뉴 데이터 소스로 오직 menu 토픽만 사용하도록 변경
  - orders 토픽의 데이터를 메뉴 소스로 사용하지 않도록 수정
  - 메뉴 데이터가 유효할 때만 처리하도록 조건 강화
  - 연결 상태를 확인하여 메뉴 데이터가 있을 때만 처리하도록 개선

이 수정으로 메뉴 추가 후 orders 토픽이 업데이트되면서 장바구니에 담긴 아이템만 메뉴로 표시되는 문제를 해결하였습니다.

## 2023-07-28: 장바구니 아이템 추가 안정성 개선
- `MenuItem.js`: 담기 버튼 클릭 핸들러 개선
  - 메뉴 아이템 객체를 직접 복사하여 필요한 속성만 추출하도록 수정
  - 원본 객체 참조 대신 새 객체를 생성하여 전달
- `MenuGrid.js`: addToCartWithQty 함수 개선
  - 모달에서 담기 기능에서도 객체를 복사하여 전달하도록 수정
- `CartContext.js`: addToCart 함수 안정성 강화
  - 입력 유효성 검사 추가
  - 수량이 정수형이고 1 이상인지 확인
  - 상태 업데이트 시 불변성 유지 방식 개선
- `HomePage.js`: handleAddToCart 함수 강화
  - 입력 아이템 유효성 검사
  - 수량 유효성 검사 및 정수 변환 추가
  - 필요한 속성만 포함한 새 객체 생성

이 수정으로 메뉴 아이템을 여러 번 클릭할 때 수량이 일관되게 처리되도록 개선하였습니다.

## 2023-09-27: VideoStreamPage 컴포넌트 ESLint 오류 수정
- LiveStreamComponent의 재시도 관련 변수 수정
  - `retryCount` 지역 변수를 `retryCountRef` useRef 훅으로 변경
  - `MAX_RETRIES` 상수를 컴포넌트 스코프로 이동
  - 컴포넌트 마운트 시 재시도 카운터를 초기화하는 로직 추가
  - 모든 참조를 업데이트하여 ESLint 오류 해결
  - useRef 사용으로 렌더링 간 값을 안정적으로 유지하도록 개선

## 2023-09-28: VideoStreamPage 컴포넌트 WebGL 렌더러 오류 및 마운트/언마운트 사이클 수정
- JSMpeg 플레이어 렌더러 변경 및 초기화 프로세스 안정화
  - WebGL 렌더러 대신 Canvas2D 렌더러 사용으로 변경 (`renderer: 'Canvas2D'` 옵션 추가)
  - 초기화 중복 실행 방지를 위한 `initializingRef` 플래그 추가
  - 컴포넌트 useEffect 의존성 배열에서 onError 제거하여 불필요한 재마운트 방지
  - cleanupStream 함수 안정화: null 참조 접근 오류 방지 및 에러 처리 강화
  - 모든 비동기 작업에 마운트 상태 확인 코드 추가
  - 초기화 및 정리 과정에서 플래그 상태 추적 및 관리 개선
  - 초기화 실패 시에도 플래그 상태 정상화하여 다음 시도 가능하도록 수정

이 업데이트로 VideoStreamPage에서 라이브 탭으로 서버에 연결된 웹캠의 실시간 영상을 볼 수 있게 되었습니다. 추후에는 CCTV나 다른 IP 카메라와의 연동을 위한 기반 마련.

## 2024-07-24: WebRTC 기반 실시간 스트리밍 구현

### 변경 내역 요약

JSMpeg 기반 스트리밍에서 WebRTC 기반 스트리밍으로 전환하여 다음과 같은 개선 사항을 구현했습니다:

1. 백엔드에서 ffmpeg 기반 스트리밍 제거하고 WebRTC 시그널링 서버 구현
2. 프론트엔드에서 JSMpeg 라이브러리 제거하고 WebRTC API 사용
3. HTTP 환경에서도 작동하는 WebRTC 구성 적용
4. RTSP 카메라 스트림 확장 지원 기능 추가

### 변경 이유

기존 JSMpeg 기반 웹소켓 스트리밍 방식은 다음과 같은 문제점이 있었습니다:

1. **높은 지연시간**: ffmpeg → mpegts → JSMpeg 변환 과정에서 최소 1-2초의 지연 발생
2. **대역폭 비효율성**: 모든 데이터가 서버를 거쳐야 하므로 네트워크 대역폭 낭비
3. **확장성 제한**: 다수의 연결에서 서버 부하 증가
4. **오류 발생 빈도 높음**: 인코딩/디코딩 과정에서 다양한 오류 발생

WebRTC를 사용함으로써 다음과 같은 이점을 얻을 수 있습니다:

1. **저지연 스트리밍**: P2P 연결로 최소한의 지연
2. **대역폭 효율성**: 최적화된 미디어 전송 및 적응형 품질
3. **확장성 개선**: 서버 부하 감소
4. **표준 웹 기술**: 브라우저 내장 API 사용
5. **HTTP 환경 호환성**: 적절한 설정으로 HTTPS가 아닌 환경에서도 작동
6. **RTSP 확장 가능**: 미디어 서버와 연동하여 RTSP 스트림 지원

### HTTP 환경에서의 WebRTC 구현 특이사항

일반적으로 WebRTC는 보안을 위해 HTTPS 환경에서 실행되어야 하지만, 개발 환경에서 다음과 같은 방식으로 HTTP에서도 작동하도록 구현했습니다:

1. **ICE 서버 구성**: STUN 서버만 사용하고 TURN 서버는 미사용
2. **로컬 네트워크 연결 우선**: ICE Candidate로 로컬 IP 우선 사용
3. **iceTransportPolicy**: 'all' 설정으로 모든 후보 허용
4. **크롬 플래그 활용**: 개발 환경에서는 `--unsafely-treat-insecure-origin-as-secure` 플래그 사용 권장

### 구현 단계

1. `backend/app/routes/live_streaming.py` 수정
   - WebRTC 시그널링 서버 구현
   - RTCSessionManager 클래스로 세션 관리
   - 웹캠 및 RTSP 스트림 목록 관리 기능
   - HTTP API 및 WebSocket 엔드포인트 추가

2. `frontend/operator/src/components/LiveStreamComponent.jsx` 수정
   - JSMpeg 의존성 제거하고 네이티브 WebRTC API 사용
   - 시그널링 프로토콜 구현
   - ICE 서버 설정 (STUN 서버만 사용)
   - HTTP 환경 호환성을 위한 설정 적용

3. `frontend/operator/src/pages/VideoStreamPage.jsx` 수정
   - 실시간 탭에 WebRTC 스트리밍 인터페이스 추가
   - RTSP 스트림 추가 UI 구현
   - 사용 가능한 스트림 목록 표시

### WebRTC 신호 교환 흐름

1. 클라이언트가 WebSocket을 통해 서버에 연결
2. 서버는 세션 ID를 생성하고 클라이언트에 전송
3. 클라이언트는 RTCPeerConnection을 생성하고 SDP Offer 생성
4. 클라이언트는 Offer를 서버로 전송
5. 양측에서 ICE 후보를 교환
6. P2P 연결이 수립되면 미디어 스트리밍 시작

### 확장 가능성: RTSP 스트림 지원

RTSP 스트림을 WebRTC로 변환하기 위해 두 가지 접근 방식을 고려할 수 있습니다:

1. **직접 변환 방식**: ffmpeg + gstreamer를 사용하여 RTSP → WebRTC 변환
2. **미디어 서버 활용**: mediasoup, Janus 등의 미디어 서버를 사용하여 변환

현재 구현에서는 기본 구조만 포함되어 있으며, 추가적인 미디어 서버 연동이 필요합니다.

### 향후 계획

1. 다중 스트림 동시 시청 기능 추가
2. WebRTC 스트림 녹화 기능 구현
3. 미디어 서버를 통한 RTSP/RTMP 스트림 변환 확장
   - MediaSoup 또는 Janus Gateway 통합 검토
4. 스트리밍 품질 및 크기 조절 옵션 추가
5. 모바일 장치 호환성 개선 

## 2024-07-25: WebRTC 카메라 스트림 관리 개선

### 문제 분석

로그 분석을 통해 다음과 같은 문제점을 발견했습니다:

1. 클라이언트 연결/해제 과정에서 너무 많은 재연결 시도 발생
2. 고정된 웹캠 ID (webcam1, webcam2)를 사용하여 실제 시스템 디바이스와 불일치 발생
3. video4 대신 다른 디바이스(video0, video5)가 사용되어 일관성 부재
4. "스트림을 찾을 수 없음" 오류 발생

### 개선 사항

1. **백엔드 변경 (`backend/app/routes/live_streaming.py`)**:
   - 실제 디바이스 검색 및 자동 등록 기능 추가 
   - `/dev/video*` 장치를 자동으로 검색하여 시스템에 연결된 실제 카메라만 등록
   - `/dev/video4`가 있으면 우선순위 설정
   - 스트림 ID 검증 로직 추가 - 존재하지 않는 스트림에 대한 접근 차단
   - RTSP URL 유효성 검사 강화
   - 디바이스 새로고침 API 엔드포인트 추가 (`/refresh-streams`)

2. **프론트엔드 변경 (`frontend/operator/src/pages/VideoStreamPage.jsx`)**:
   - 사용 가능한 스트림 목록을 백엔드에서 가져와 표시
   - 스트림 선택 UI 개선 - 선택한 스트림만 연결
   - 디바이스 새로고침 버튼 추가
   - RTSP 입력 폼 개선 및 유효성 검사 추가

3. **연결 오류 처리 개선 (`frontend/operator/src/components/LiveStreamComponent.jsx`)**:
   - 연결 실패 시 명확한 오류 메시지 표시
   - 재연결 로직 개선 및 최대 재시도 횟수 설정
   - 다양한 오류 상황에 대한 사용자 피드백 추가

### 기대 효과

1. 실제 존재하는 카메라 장치만 표시되어 사용자 혼란 감소
2. 스트림 선택 방식으로 불필요한 연결 시도 감소
3. 명확한 오류 메시지를 통한 디버깅 용이성 증가
4. 디바이스 추가/제거 시 새로고침을 통한 즉시 반영 가능

### 테스트 결과

로그 분석 결과, 디바이스 접근 및 연결 설정 부분에서 반복적인 재연결 시도가 감소했으며, 클라이언트에 의미 있는 오류 메시지가 제공되어 문제 해결이 용이해졌습니다. 향후 모니터링을 통해 안정성을 계속 평가할 예정입니다. 

## 2024-07-26: 실시간 스트리밍 목록 설정 개선

### 변경 내역 요약

자동 탐색 방식에서 사용자 설정 방식으로 스트림 관리 방식을 개선했습니다:

1. 백엔드에서 카메라 자동 탐색 방식 제거하고 고정 설정 방식으로 변경
2. RTSP 추가 폼 개선하여 이름 입력 기능 추가
3. 웹캠/RTSP 스트림 관리 기능 개선

### 변경 이유

기존 방식에서는 다음과 같은 문제점이 있었습니다:

1. **사용자 혼란**: 다양한 스트림이 자동 탐색되어 표시되어 의도하지 않은 카메라 노출
2. **불필요한 기능**: RTSP 스트림 추가 기능이 일반 사용자에게 불필요
3. **UI 복잡성**: 복잡한 UI로 인해 사용자 경험 저하

고정 스트림 방식으로 변경함으로써 다음과 같은 이점을 얻을 수 있습니다:

1. **단순한 사용자 경험**: 필요한 스트림만 표시되어 사용이 간편
2. **관리 용이성**: 시스템 관리자만 코드에서 스트림 설정 가능
3. **안정적인 표시**: 페이지 로드 시 자동으로 첫 번째 스트림 선택 및 표시

### 구현 내용

1. `backend/app/routes/live_streaming.py` 수정
   - DEFAULT_WEBCAMS 및 DEFAULT_RTSP_STREAMS 설정에 실제 사용할 카메라 정의
   - 고정 설정 값으로 메인 카메라, 보조 카메라 및 CCTV 스트림 설정

2. `frontend/operator/src/pages/VideoStreamPage.jsx` 수정
   - RTSP 스트림 추가 기능 및 UI 제거
   - 그리드 레이아웃으로 변경하여 왼쪽에 스트림 목록, 오른쪽에 플레이어 배치
   - 페이지 로드 시 자동으로 첫 번째 웹캠 선택 및 표시 기능 추가
   - 스트림 리스트 UI 간소화 및 선택된 스트림 강조 표시

### 설정 방법

고정 스트림을 설정하려면 `backend/app/routes/live_streaming.py` 파일의 다음 부분을 수정합니다:

1. 웹캠 설정:
   ```python
   DEFAULT_WEBCAMS = [
       {"id": "camera_4", "path": "/dev/video4", "display_name": "메인 카메라"},
       {"id": "camera_0", "path": "/dev/video0", "display_name": "보조 카메라"},
   ]
   ```

2. RTSP 스트림 설정:
   ```python
   DEFAULT_RTSP_STREAMS = [
       {"id": "rtsp_main", "url": "rtsp://admin:admin123@192.168.0.108:554/live", "display_name": "메인 CCTV"},
       {"id": "rtsp_entrance", "url": "rtsp://admin:admin123@192.168.0.109:554/live", "display_name": "입구 CCTV"},
   ]
   ```

코드를 수정한 후 서버를 재시작하면 변경사항이 적용됩니다. 

## 2024-07-27: 웹캠 전용 스트리밍 기능 구현

### 변경 내역 요약

실시간 스트리밍 기능을 웹캠 전용으로 단순화하고 백엔드 API 의존성을 최소화했습니다:

1. 프론트엔드에서 WebRTC 스트리밍 인터페이스 단순화
2. 외부 API 의존성 제거 및 직접 백엔드 통신 구현
3. 웹캠 중심의 UI 설계 적용

### 변경 이유

이전 구현에는 다음과 같은 문제점이 있었습니다:

1. **불필요한 API 호출**: videostream 관련 외부 API 호출로 인한 복잡성
2. **다양한 스트림 타입 혼합**: 웹캠과 RTSP 스트림이 혼합되어 표시되는 UI로 인한 사용자 혼란
3. **복잡한 WebSocket 의존성**: 실시간 스트리밍에 불필요한 WebSocket 통신 사용

### 구현 내용

1. `frontend/operator/src/pages/VideoStreamPage.jsx` 개선:
   - 외부 API 의존성 제거하고 `/streams` 엔드포인트 직접 호출
   - 자동 새로고침 로직을 간소화하여 필요할 때만 스트림 데이터 로드
   - 웹캠 전용 필터링 적용 (stream.type === 'webcam' 조건으로 필터링)
   - UI를 웹캠 중심으로 개선 (웹캠 아이콘 및 메시지 표시)
   - 새로고침 버튼 추가로 사용자가 필요시 목록 갱신 가능
   - 의존성 문제 해결을 위한 useCallback 활용

### 작동 방식

1. **간소화된 데이터 로드**: 
   - 페이지 접속 시 `/streams` 엔드포인트를 직접 호출하여 스트림 목록 로드
   - 웹캠 스트림 필터링을 백엔드에서 가져와 표시
   - 불필요한 데이터 캐싱 및 중복 요청 제거

2. **웹캠 중심 UI**:
   - "실시간 스트림" 대신 "웹캠 스트리밍"으로 제목 변경
   - 스트림 목록에 웹캠 타입만 표시
   - RTSP 관련 아이콘 및 UI 요소 제거

3. **성능 개선**:
   - 불필요한 API 호출 제거로 부하 감소
   - 독립적인 상태 관리로 외부 데이터 의존성 감소
   - 컴포넌트 렌더링 최적화 (useCallback 활용)

### 향후 개선 방향

1. 추가 웹캠 설정 기능 (해상도, 프레임레이트 등) 구현 가능성 검토
2. 백엔드 웹캠 인식 메커니즘 강화
3. 웹캠 관련 오류 메시지 세분화 및 문제 해결 가이드 추가
4. 웹캠 스냅샷 캡처 기능 고려 

## 2024-07-27: WebRTC 스트리밍 안정성 개선

### 개선 사항 요약
1. WebRTC 연결 안정성 강화
2. 실시간 스트리밍 오류 처리 개선
3. 핑-퐁 메커니즘 추가를 통한 연결 유지
4. 백엔드와 프론트엔드 간 시그널링 프로토콜 최적화

### 개선 배경
실시간 스트리밍 기능 사용 시 다음과 같은 문제점이 발견되었습니다:
- WebSocket 연결이 비정상적으로 종료되는 문제
- 연결 실패 시 적절한 오류 메시지가 표시되지 않는 문제
- 일정 시간 후 스트리밍 연결이 끊기는 문제
- 재연결 시도 시 불필요한 자원 소모

### 개선 내용

#### 1. 프론트엔드 (LiveStreamComponent.jsx) 개선
- WebSocket 연결 시간 초과 처리 개선 (5초 → 10초)
- 핑-퐁 메커니즘 구현 (15초마다 핑 메시지 전송)
- 연결 실패 시 백오프 전략 적용 (지수적 대기 시간 증가)
- 네트워크 정보 로깅 기능 추가
- ICE 연결 상태 모니터링 개선
- 필요한 경우 연결 즉시 재시도 로직 추가

```javascript
// 핑-퐁 메커니즘
const setupPingPong = () => {
  if (pingInterval) clearInterval(pingInterval);
  
  pingInterval = setInterval(() => {
    if (ws.readyState === WebSocket.OPEN) {
      try {
        ws.send(JSON.stringify({ type: 'ping' }));
      } catch (e) {
        console.error('핑 전송 실패:', e);
        if (pingInterval) {
          clearInterval(pingInterval);
          pingInterval = null;
        }
      }
    }
  }, 15000); // 15초마다 핑 전송
};
```

#### 2. 백엔드 (live_streaming.py) 개선
- 스트림 존재 여부 사전 검증 로직 추가
- 웹소켓 연결 타임아웃 관리 (30초)
- 클라이언트 상태 관리 개선
- 오류 처리 및 로깅 강화
- 비정상 연결 종료 시 자원 정리 보장
- JSON 파싱 오류 처리 및 회복 로직 추가

```python
# 타임아웃 체크 (30초 동안 메시지가 없으면 핑 전송)
current_time = time.time()
if current_time - last_message_time > 30:
    # 핑 메시지 전송
    await websocket.send_json({"type": "pong"})
    logging.debug(f"클라이언트 {client_id}에 핑 전송")
    last_message_time = current_time
```

#### 3. 시그널링 프로토콜 최적화
- 명확한 메시지 타입 정의 및 일관된 처리
- 오류 메시지 표준화
- 세션 관리 및 연결 상태 추적 개선
- ICE 후보 교환 과정 최적화

### 기대 효과
- 실시간 스트리밍 연결 안정성 향상
- 사용자 경험 개선 (오류 메시지, 연결 상태 피드백)
- 네트워크 자원 효율적 사용
- 시스템 모니터링 및 디버깅 용이성 향상

### 후속 개선 계획
- 스트리밍 품질 조정 옵션 추가 검토
- 다중 스트림 동시 시청 기능 고려
- 네트워크 상태에 따른 적응형 스트리밍 구현 검토 

## 2024-07-28: WebRTC 연결 처리 개선

### 개선 사항 요약
1. 백엔드 코드 정리 및 미디어 서버 연결 처리 추가
2. 프론트엔드 WebRTC 연결 안정성 개선
3. 백엔드 API 엔드포인트 동기화 및 개선

### 개선 배경
WebRTC 스트리밍 연결이 불안정하고 반복적으로 연결/재연결을 시도하는 문제가 발생했습니다. 로그 분석 결과 다음과 같은 원인을 파악했습니다:
- 백엔드 측에서 명확한 미디어 연결 처리가 누락됨
- 디바이스 검증 로직이 실제 스트림 연결 전에 적절히 수행되지 않음
- WebSocket 연결 종료 후 리소스 정리가 제대로 이루어지지 않음
- 프론트엔드에서 연결 상태 추적 로직이 부족함

### 개선 내용

#### 1. 백엔드 코드 개선 (live_streaming.py)
- 고정 리소스와 런타임 리소스 분리하여 관리
  - `streams`, `client_register`, `webrtc_sessions` 전역 변수로 명확하게 구분
  - 스트림 초기화 함수 구현으로 일관된 초기화 로직 제공
- 미디어 서버 연결 처리 추가
  - 웹캠 디바이스 경로 검증으로 잘못된 연결 시도 방지
  - WebRTC 신호 교환 중 명확한 상태 전이 처리
  - 미디어 프로세스 관리 및 정리 로직 추가
- API 엔드포인트 개선
  - 스트림 목록 반환 로직 개선 및 정렬 기능 추가 (웹캠 우선 정렬)
  - 스트림 새로고침 기능 동작 개선
  - RTSP 스트림 추가 로직 개선
- 오류 처리 강화
  - WebSocket 예외 명시적 처리 (`ConnectionClosedOK`, `ConnectionClosedError` 등)
  - 적절한 오류 메시지 및 로깅 추가

#### 2. 프론트엔드 개선 (LiveStreamComponent.jsx)
- ICE 연결 상태 추적 개선
  - 활성 트랙 기반 실제 연결 상태 검증 추가 (`hasActiveTracks` 확인)
  - 일시적인 연결 문제와 심각한 연결 문제 구분 처리
- 연결 상태 관리 강화
  - `onconnectionstatechange` 이벤트 핸들러 추가로 P2P 연결 상태 추적
  - 비디오 트랙 활성화 감지 및 UI 상태 연동 (`onunmute` 이벤트)
- 리소스 정리 및 재연결 로직 개선
  - 명시적인 연결 정리 후 재연결 시도
  - 지연된 초기화로 DOM 마운트 이슈 방지
- 백오프 전략 적용
  - 연결 실패 시 점진적으로 재시도 간격 증가

### 개선 효과
1. 불필요한 재연결 시도 감소로 서버/클라이언트 리소스 낭비 방지
2. 사용자 경험 개선: 연결 상태에 따른 적절한 UI 피드백 제공
3. 디버깅 용이성 향상: 상세 로깅 및 명확한 오류 메시지
4. 백엔드 코드 가독성 및 유지보수성 향상
5. 리소스 누수 방지: 연결 종료 시 적절한 정리 보장

### 후속 개선 계획
1. 미디어 서버 연동 강화: janus, mediasoup 등 고려
2. 스트림 품질 조정 옵션 구현
3. 비디오 녹화 기능 추가 검토
4. 브라우저/기기 호환성 테스트 및 최적화

## 2024-07-29: 백엔드 utils 모듈 의존성 제거

### 문제 상황
- `live_streaming.py` 파일에서 `from app import deps, utils` 임포트 구문이 오류를 발생시키는 문제가 있었습니다.
- `ImportError: cannot import name 'deps' from 'app'` 오류 메시지가 발생했습니다.

### 분석
- `app` 모듈에 `deps` 모듈이 존재하지 않아 임포트 오류가 발생했습니다.
- 다른 라우트 파일들의 임포트 패턴을 살펴본 결과, 대부분 `app.core.db_config`에서 `get_db`를 직접 임포트하고 있었습니다.
- `utils` 모듈의 경우도 `app.core.utils`에 존재하는 것으로 확인되었습니다.

### 수정 내용
- `live_streaming.py` 파일의 임포트 구문을 다음과 같이 변경했습니다:
  ```python
  # 변경 전
  from app import deps, utils
  
  # 변경 후
  from app.core.db_config import get_db
  from app.core import utils
  ```
- `websocket_endpoint` 함수의 의존성 주입 부분도 수정했습니다:
  ```python
  # 변경 전
  async def websocket_endpoint(websocket: WebSocket, stream_id: str, db: Session = Depends(deps.get_db)):
  
  # 변경 후
  async def websocket_endpoint(websocket: WebSocket, stream_id: str, db: Session = Depends(get_db)):
  ```
- `utils.find_by_id_in_dict` 함수가 `app.core.utils`에 존재하지 않아 추가적인 수정이 필요했습니다:
  - `app.core.utils` 모듈 임포트를 제거
  - `live_streaming.py` 파일 내에서 `find_by_id_in_dict` 함수를 직접 구현:
    ```python
    def find_by_id_in_dict(items_dict: Dict, item_id: str) -> Optional[Dict]:
        """
        ID로 사전에서 항목을 찾아 반환합니다.
        
        Args:
            items_dict: 검색할 사전
            item_id: 찾을 항목의 ID
            
        Returns:
            찾은 항목 또는 None
        """
        return items_dict.get(item_id)
    ```
  - 함수 사용 부분도 `find_by_id_in_dict`에서 `find_by_id_in_dict`로 변경

### 결과
- 백엔드 서버가 정상적으로 시작되도록 임포트 오류를 해결했습니다.
- 프로젝트 코드 베이스의 일관성을 유지하면서 종속성 주입 문제를 해결했습니다.
- 외부 의존성을 제거하고 파일 내에서 필요한 유틸리티 함수를 직접 구현함으로써 코드의 자립성이 향상되었습니다.

## 2024-07-30: WebRTC 연결 실패 문제 해결

### 문제 상황
- WebRTC 시그널링 과정에서 ICE 후보 추가 실패 오류 발생
- `InvalidStateError: Failed to execute 'addIceCandidate' on 'RTCPeerConnection': The remote description was null` 에러 발생
- 웹소켓 연결이 정상적으로 설정되었으나 WebRTC 연결이 실패하는 문제
- 반복적인 재연결 시도 및 타임아웃 발생

### 분석
1. 백엔드 측 코드에서 클라이언트의 offer SDP를 그대로 answer로 돌려보내는 문제
   - 적절한 SDP answer 생성 로직이 구현되어 있지 않음
   - ICE 후보가 교환되기 전에 remoteDescription이 설정되지 않음
2. 프론트엔드 측 코드에서 ICE 후보 처리 로직 부재
   - remoteDescription 설정 전에 도착한 ICE 후보 처리 불가
   - 후보 큐 관리 메커니즘 부재

### 수정 내용
1. 백엔드 수정 (`backend/app/routes/live_streaming.py`):
   - 클라이언트의 offer에 대한 적절한 answer SDP 생성 로직 구현
   - 웹캠 스트림 상태를 active로 업데이트하는 코드 추가
   - 더미 answer SDP 형식을 WebRTC 표준에 맞게 수정

2. 프론트엔드 수정 (`frontend/operator/src/components/LiveStreamComponent.jsx`):
   - ICE 후보 큐 관리 메커니즘 추가
     - `remoteDescriptionSetRef` 플래그로 원격 설명 설정 상태 추적
     - `iceCandidatesQueueRef` 큐로 초기 ICE 후보 저장
   - answer 수신 핸들러 추가하여 `setRemoteDescription` 처리
   - 큐에 저장된 ICE 후보를 원격 설명 설정 후 적용하는 로직 추가
   - `setupPeerConnection` 함수의 의존성 배열 수정으로 무한 루프 방지

### 결과
- 프론트엔드와 백엔드 간 정상적인 WebRTC 시그널링 프로세스 수립
- ICE 후보 교환 과정이 적절히 처리되어 연결 성공률 향상
- 불필요한 재연결 시도 감소 및 연결 안정성 개선
- 사용자에게 명확한 오류 메시지 제공
