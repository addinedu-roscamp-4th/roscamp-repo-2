# 프로세스 변경 내역

## 2024-05-19: 키오스크 청소 완료 버튼 기능 추가

### 변경 사항
1. 키오스크에서 테이블이 청소 중일 때 청소 완료 버튼 표시 기능 추가
   - 테이블 상태가 CLEANING일 때 화면 중앙에 청소 완료 버튼 표시
   - 버튼 클릭 시 테이블 상태를 AVAILABLE로 변경
   - 웹소켓을 통한 실시간 상태 업데이트
2. 청소 완료 버튼 UI 개선
   - 어두운 오버레이 배경 추가 (rgba(0, 0, 0, 0.85))
   - 안내 메시지 추가
   - 버튼 디자인 개선

### 수정된 파일
- `robodine_service/frontend/kiosk/src/api/orderApi.js`
  - 테이블 상태 업데이트 함수 `updateTableStatus` 추가
- `robodine_service/frontend/kiosk/src/components/CleaningButton.jsx` (신규)
  - 청소 완료 버튼 컴포넌트 구현
  - 오버레이 및 안내 메시지 추가
- `robodine_service/frontend/kiosk/src/App.js`
  - 청소 완료 버튼 컴포넌트 추가

### 패키지 설치
- styled-components: CSS-in-JS 스타일링을 위한 패키지 설치

### 기술 스택
- React
- WebSocket
- Styled Components
- Axios 

## 로봇 디버깅 UI 도구 패키지 구성

ROS 2 패키지로 로봇 디버깅 UI 도구를 구성했습니다. C++/Qt 기반의 UI 도구로, Jetcobot 및 Pinky 로봇을 모니터링하고 제어할 수 있습니다.

### 생성한 파일 목록

- `robot_debugger/src/robot_debugger_ui/package.xml`: 패키지 메타데이터
- `robot_debugger/src/robot_debugger_ui/CMakeLists.txt`: CMake 빌드 파일
- `robot_debugger/src/robot_debugger_ui/launch/robot_debugger_ui_launch.py`: 런치 파일
- `robot_debugger/src/robot_debugger_ui/include/robot_debugger_ui/main_window.hpp`: 메인 윈도우 헤더
- `robot_debugger/src/robot_debugger_ui/include/robot_debugger_ui/topic_manager.hpp`: 토픽 관리자 헤더
- `robot_debugger/src/robot_debugger_ui/include/robot_debugger_ui/message_store.hpp`: 메시지 저장소 헤더
- `robot_debugger/src/robot_debugger_ui/include/robot_debugger_ui/configuration_manager.hpp`: 설정 관리자 헤더
- `robot_debugger/src/robot_debugger_ui/include/robot_debugger_ui/plugin_manager.hpp`: 플러그인 관리자 헤더
- `robot_debugger/src/robot_debugger_ui/ui/main_window.ui`: 메인 윈도우 UI 파일
- `robot_debugger/src/robot_debugger_ui/ui/dashboard.ui`: 대시보드 UI 파일
- `robot_debugger/src/robot_debugger_ui/ui/jetcobot_tab.ui`: Jetcobot 탭 UI 파일
- `robot_debugger/src/robot_debugger_ui/ui/pinky_tab.ui`: Pinky 탭 UI 파일
- `robot_debugger/src/robot_debugger_ui/src/main.cpp`: 메인 소스 파일
- `robot_debugger/src/robot_debugger_ui/src/main_window.cpp`: 메인 윈도우 구현 파일
- `robot_debugger/src/robot_debugger_ui/src/topic_manager.cpp`: 토픽 관리자 구현 파일
- `robot_debugger/src/robot_debugger_ui/src/message_store.cpp`: 메시지 저장소 구현 파일
- `robot_debugger/src/robot_debugger_ui/src/configuration_manager.cpp`: 설정 관리자 구현 파일
- `robot_debugger/src/robot_debugger_ui/src/plugin_manager.cpp`: 플러그인 관리자 구현 파일
- `robot_debugger/src/robot_debugger_ui/resources/icons.qrc`: 아이콘 리소스 파일
- `robot_debugger/README.md`: 패키지 설명 파일

### 빌드 오류 수정

다음과 같은 빌드 오류를 수정했습니다:

1. `PluginInterface` 클래스의 `Q_OBJECT` 매크로 문제:
   - `PluginInterface`는 Qt의 MOC 처리를 필요로 하지 않으므로 일반 인터페이스로 변경
   - `qobject_cast` 대신 `dynamic_cast`를 사용하여 객체 캐스팅 처리

2. `MainWindow` 클래스의 UI 포인터 처리 문제:
   - `ui_` 포인터를 올바르게 초기화하고 소멸자에서 삭제 로직 수정
   - `Ui::MainWindow` 전방 선언 추가

3. 리소스 파일 처리:
   - 아이콘 파일이 없는 상태에서도 빌드 가능하도록 리소스 설정 수정

4. CMakeLists.txt 파일 수정:
   - 아직 구현되지 않은 패널 파일 선언 제거
   - 헤더 파일 설치 규칙 추가

5. Qt MOC (Meta-Object Compiler) 관련 오류 해결:
   - 소스 및 헤더 파일 목록 명시적 설정
   - `qt5_wrap_cpp()` 함수로 명시적 MOC 파일 생성 설정
   - 빌드 디렉토리를 include 경로에 추가
   - 자동 MOC 처리 옵션 활성화 (CMAKE_AUTOMOC_RELAXED_MODE, CMAKE_AUTORCC, CMAKE_AUTOUIC) 

### 빌드 성공

MOC 관련 빌드 오류를 해결하고 robot_debugger 디렉토리 내에서 다음 명령을 실행하여 패키지를 성공적으로 빌드했습니다:

```bash
cd /home/addinedu/dev_ws/roscamp-repo-2/robot_debugger && colcon build --packages-select robot_debugger_ui
```

빌드 성공 결과:
```
Starting >>> robot_debugger_ui
Finished <<< robot_debugger_ui [0.16s]                
Summary: 1 package finished [0.32s]
```

### UI 개선

다음과 같은 UI 개선 작업을 수행했습니다:

1. Qt UI 파일 로드 기능 추가:
   - Qt UiLoader를 사용하여 .ui 파일에서 UI 직접 로드
   - ROS 2 패키지의 share 디렉토리에서 UI 파일 경로 설정

2. 탭 전환 버튼 추가:
   - 대시보드, Jetcobot, Pinky 패널 간 전환을 위한 툴바 추가
   - 각 탭 버튼 클릭 시 해당 패널로 전환되도록 구현

3. Qt 의존성 추가:
   - Qt5UiTools 패키지 의존성 추가
   - UI 파일 동적 로드를 위한 라이브러리 링크 

### UI 파일 경로 문제 해결

UI 파일을 찾지 못하는 문제를 다음과 같이 해결했습니다:

1. UI 파일 경로 찾기 강화:
   - AMENT_PREFIX_PATH 환경 변수의 각 경로에서 UI 파일을 순차적으로 검색
   - 상대 경로 대체 옵션 추가 (../share 및 share 디렉토리)
   - 파일 존재 여부 확인 후 유효한 경로만 사용

2. 기본 UI 개선:
   - UI 파일을 찾지 못할 경우 표시할 기본 UI 개선
   - 각 탭에 기본 레이블과 레이아웃 추가
   - 경로 정보 및 로딩 상태를 상태바에 표시

3. 오류 처리 강화:
   - UI 파일 로드 실패 시 오류 메시지에 파일 경로 정보 추가
   - 로드 실패 탭에 명확한 오류 표시 