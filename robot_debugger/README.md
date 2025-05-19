# 로봇 디버깅 UI 도구

ROS 2 기반 로봇 디버깅을 위한 UI 도구입니다. Jetcobot(2대)과 Pinky(3대) 로봇을 실시간으로 모니터링하고 제어할 수 있습니다.

## 주요 기능

- 사용자 지정 도메인 및 네임스페이스 패턴으로 토픽 구독
- 항상-ON 구독으로 메시지 손실 없이 저장
- 필요한 토픽만 UI에 노출
- 로봇별 특화 기능:
  - Jetcobot: 좌표 제어, 조인트 티칭, 템플릿 저장 및 재생
  - Pinky: 멀티 로봇 동시 제어, 맵 기반 경로 지정 및 실행
- 로그 레벨 필터링, 이력 보존, 이벤트 복원

## 폴더 구조

```
robot_debugger/
├── src/
│   └── robot_debugger_ui/      # ROS 2 패키지
│       ├── CMakeLists.txt      # CMake 빌드 파일
│       ├── package.xml         # 패키지 정보
│       ├── src/                # 소스 코드
│       │   ├── main.cpp        # 메인 함수
│       │   ├── main_window.cpp # 메인 윈도우 구현
│       │   ├── topic_manager.cpp # 토픽 관리자
│       │   └── ...
│       ├── include/robot_debugger_ui/ # 헤더 파일
│       │   ├── main_window.hpp # 메인 윈도우 헤더
│       │   ├── topic_manager.hpp # 토픽 관리자 헤더
│       │   └── ...
│       ├── ui/                 # Qt Designer UI 파일
│       │   ├── main_window.ui  # 메인 윈도우 UI
│       │   ├── dashboard.ui    # 대시보드 UI
│       │   └── ...
│       ├── resources/          # 리소스 파일
│       │   └── icons.qrc       # 아이콘 리소스
│       └── launch/             # 런치 파일
│           └── robot_debugger_ui_launch.py # 런치 스크립트
└── README.md                   # 이 파일
```

## 모듈 설명

### MainWindow

로봇 디버깅 UI의 메인 윈도우를 담당합니다. 툴바, 상태바, 사이드바, 탭별 페이지를 포함합니다.

### TopicManager

ROS 2 토픽을 관리하는 클래스입니다. 지정된 패턴에 맞는 토픽을 스캔하고 구독합니다.

### MessageStore

구독한 토픽 메시지를 저장하는 클래스입니다. 토픽별로 순환 버퍼를 유지합니다.

### ConfigurationManager

도메인, 네임스페이스, QoS, 로그 설정 등을 관리하는 클래스입니다.

### PluginManager

확장 모듈을 로드하고 관리하는 클래스입니다.

## UI 탭 설명

### 대시보드 탭
- Fleet Overview: 로봇 카드
- Global Log Viewer: 로그 뷰어
- Event Timeline: 이벤트 타임라인
- Quick-Add Panel: 자주 쓰는 패널 단축 추가

### Jetcobot 탭
- 좌표 제어 패널
- 티칭 패널
- 궤적 편집 패널
- 템플릿 관리 패널

### Pinky 탭
- 맵 플래너 패널
- 경로 그리기 패널
- 경로 시각화 패널
- 작업 큐 패널

## 빌드 및 실행 방법

### 의존성 설치

```bash
sudo apt-get update
sudo apt-get install -y qtbase5-dev libqt5widgets5 libqt5gui5 libqt5core5a
sudo apt-get install -y ros-$ROS_DISTRO-rclcpp ros-$ROS_DISTRO-std-msgs
sudo apt-get install -y ros-$ROS_DISTRO-qt-gui-cpp ros-$ROS_DISTRO-geometry-msgs
sudo apt-get install -y ros-$ROS_DISTRO-nav-msgs ros-$ROS_DISTRO-sensor-msgs
```

### 빌드

```bash
cd robot_debugger
colcon build --packages-select robot_debugger_ui
```

### 실행

```bash
source install/setup.bash
ros2 launch robot_debugger_ui robot_debugger_ui_launch.py
```

### 명령행 옵션

```bash
# 도메인 패턴 지정
ros2 launch robot_debugger_ui robot_debugger_ui_launch.py domains:="[10, 20]"

# 네임스페이스 패턴 지정
ros2 launch robot_debugger_ui robot_debugger_ui_launch.py namespaces:="[\"jetcobot_1\", \"pinky_1\"]"

# 로그 레벨 지정
ros2 launch robot_debugger_ui robot_debugger_ui_launch.py log_level:=debug
```

## 라이센스

Apache License 2.0 