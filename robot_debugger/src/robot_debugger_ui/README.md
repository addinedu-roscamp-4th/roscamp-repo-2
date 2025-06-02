# 로봇 디버거 UI (Robot Debugger UI)

로봇 디버거 UI는 Jetcobot 및 Pinky 로봇을 모니터링하고 디버깅하기 위한 Qt 기반 GUI 도구입니다. ROS 2를 사용하여 로봇의 상태를 실시간으로 확인하고, 문제를 진단할 수 있습니다.

## 1. 주요 기능

- **멀티 로봇 모니터링**: 여러 Jetcobot 및 Pinky 로봇을 동시에 모니터링
- **대시보드**: 플릿 오버뷰, 글로벌 로그, 이벤트 탭으로 구성된 중앙 모니터링 시스템
- **로봇별 상세 정보**: 각 로봇 유형(Jetcobot, Pinky)에 대한 세부 정보 및 제어 기능
- **토픽 모니터링**: ROS 2 토픽을 자동으로 감지하고 구독하여 데이터 시각화
- **로그 시스템**: 로봇 상태 및 이벤트에 대한 로그 기록 및 표시
- **플러그인 시스템**: 확장 가능한 플러그인 아키텍처를 통한 기능 확장

## 2. 시스템 아키텍처

### 2.1 주요 컴포넌트

- **MainWindow**: 메인 애플리케이션 창 및 UI 관리
- **TopicManager**: ROS 2 토픽 관리 및 메시지 처리
- **ConfigurationManager**: 설정 저장 및 로드
- **MessageStore**: 메시지 히스토리 및 로그 관리
- **PluginManager**: 플러그인 로드 및 관리
- **Panel Interface**: 대시보드에 추가할 수 있는 패널 위젯의 인터페이스

### 2.2 디렉토리 구조

```
robot_debugger_ui/
├── include/               # 헤더 파일
│   └── robot_debugger_ui/
│       ├── main_window.hpp
│       ├── topic_manager.hpp
│       ├── configuration_manager.hpp
│       ├── message_store.hpp
│       ├── plugin_manager.hpp
│       ├── panel_interface.hpp
│       └── panels/        # 패널 인터페이스 구현
├── src/                   # 소스 파일
│   ├── main.cpp           # 메인 진입점
│   ├── main_window.cpp    # 메인 윈도우 구현
│   ├── topic_manager.cpp  # 토픽 관리자 구현
│   ├── message_store.cpp  # 메시지 저장소 구현
│   ├── plugin_manager.cpp # 플러그인 관리자 구현
│   ├── configuration_manager.cpp # 설정 관리자 구현
│   └── panels/           # 패널 구현
├── ui/                    # Qt UI 파일
│   ├── main_window.ui     # 메인 윈도우 UI
│   ├── dashboard.ui       # 대시보드 UI
│   ├── jetcobot_tab.ui    # Jetcobot 탭 UI
│   └── pinky_tab.ui       # Pinky 탭 UI
├── resources/             # 리소스 파일 (아이콘, 이미지 등)
├── launch/                # ROS 2 런치 파일
│   └── robot_debugger_ui_launch.py
├── external/              # 외부 라이브러리
├── CMakeLists.txt         # CMake 빌드 설정
└── package.xml            # ROS 2 패키지 정보
```

## 3. 주요 클래스 설명

### 3.1 MainWindow

메인 윈도우는 애플리케이션의 핵심 UI 컴포넌트로, 다음과 같은 기능을 제공합니다:
- 탭 기반 인터페이스 (대시보드, Jetcobot, Pinky, 설정)
- 로봇 목록 관리 및 표시
- 로봇 상태 모니터링 및 제어
- 이벤트 로깅 및 표시

### 3.2 TopicManager

ROS 2 토픽을 관리하고 구독/발행하는 클래스입니다:
- 특정 도메인 ID 및 네임스페이스에 맞는 토픽 자동 감지
- 다양한 메시지 타입 지원을 위한 동적 구독 생성
- 토픽 메시지 수신 및 발행 기능
- 토픽 상태 모니터링 및 알림

### 3.3 MessageStore

메시지와 로그를 저장하고 관리하는 클래스입니다:
- 토픽별 메시지 히스토리 관리
- 로그 레벨별 필터링 기능
- 시간 기반 메시지 쿼리 기능
- 로그 파일 저장 및 로드

### 3.4 PluginManager

플러그인 시스템을 관리하는 클래스입니다:
- 플러그인 동적 로드 및 언로드
- 플러그인 활성화/비활성화 관리
- 플러그인 설정 관리
- 플러그인 인터페이스 제공

### 3.5 PanelInterface

대시보드에 추가할 수 있는 패널의 인터페이스를 정의합니다:
- 패널 ID, 제목, 설명 정보 제공
- 패널 초기화 및 설정 관리
- 패널 상태 업데이트 및 종료 처리

## 4. 사용 방법

### 4.1 빌드 및 실행

```bash
# 워크스페이스로 이동
cd ~/dev_ws/roscamp-repo-2

# 빌드
colcon build --packages-select robot_debugger_ui

# 환경 설정
source install/setup.bash

# 실행
ros2 launch robot_debugger_ui robot_debugger_ui_launch.py
```

### 4.2 명령행 옵션

```bash
# 특정 도메인 및 네임스페이스 지정
ros2 run robot_debugger_ui robot_debugger_ui_node --domains "10,20" --namespaces "jetcobot_1,pinky_1"

# 로그 레벨 설정
ros2 run robot_debugger_ui robot_debugger_ui_node --log-level debug
```

### 4.3 런치 파일 파라미터

런치 파일을 통해 다음 파라미터를 설정할 수 있습니다:
- `domains`: 모니터링할 ROS 도메인 ID 목록
- `namespaces`: 모니터링할 로봇 네임스페이스 목록
- `log_level`: 로그 레벨 설정 (debug, info, warn, error, fatal)
- `qos_reliability`: QoS 신뢰성 설정
- `history_depth`: 메시지 히스토리 저장 깊이

## 5. 업데이트 내역

### 2023-12-01 업데이트

- 대시보드 탭을 플릿 오버뷰, 글로벌 로그, 이벤트 탭으로 재구성
- 로봇 카드 레이아웃을 수직 구성으로 변경
- 로봇 유형별 체크박스 섹션 구현
- 글로벌 로그 및 이벤트 탭 추가
- UI 버그 수정 및 개선

## 6. 개발 계획

- **도메인 브릿지 기능 구현**: 여러 ROS 2 도메인 간의 토픽 자동 중계 기능 추가
  - 도메인 간 토픽 매핑 UI 제공
  - 메시지 타입 자동 변환 지원
  - 토픽 필터링 및 변환 규칙 설정 기능
  - QoS 프로파일 조정 기능 