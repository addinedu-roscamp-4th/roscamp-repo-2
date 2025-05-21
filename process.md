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

### 2023-11-05: 빌드 오류 수정

다음과 같은 빌드 관련 문제를 해결했습니다:

1. QCustomPlot 라이브러리 처리:
   - 라이브러리 파일이 없을 때도 컴파일되도록 조건부 처리 추가
   - CMakeLists.txt에 조건부 컴파일 플래그 추가 (`NO_QCUSTOMPLOT`)
   - BatteryGaugePanel 클래스가 QCustomPlot 없이도 기본 기능 제공

2. 클래스 구현과 헤더 불일치 수정:
   - TopicManager: 헤더 선언에 맞게 구현 코드 수정 (메서드 구현 추가)
   - PluginManager: 헤더 선언에 맞게 구현 코드 수정 (메서드 구현 추가)
   - 실제 구현이 선언과 다른 메서드들을 일치시킴

3. 미사용 매개변수 경고 제거:
   - `updateBatteryStatus` 메서드에서 `topic_name` 매개변수 경고 제거
   - `onTopicsChanged` 메서드에서 `topics` 매개변수 경고 제거

4. ROS 2 콜백 함수 시그니처 수정:
   - 토픽 메시지 콜백 함수의 매개변수 타입 수정
   - `const std::shared_ptr<T>&` 대신 `std::shared_ptr<T>` 사용
   - `genericMessageCallback` 메서드의 매개변수 타입도 일치하도록 수정

### 2023-11-05: 빌드 성공

모든 오류를 수정하여 성공적으로 빌드를 완료했습니다. 주요 변경사항:

1. QCustomPlot 없이도 기본 배터리 게이지 기능 제공
2. ConfigurationManager와 연동하여 도메인 정보 처리
3. ROS 2 콜백 인터페이스 호환성 개선
4. 헤더와 구현 일치화로 컴파일러 오류 해결

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

### 기능 개선 및 확장

기획안에 맞춰 다음과 같은 기능을 추가 및 개선했습니다:

1. 코어 컴포넌트 강화:
   - ConfigurationManager: 도메인 및 네임스페이스 패턴, QoS 설정, 레이아웃 관리 기능 확장
   - MessageStore: 토픽별 메시지 저장, 시간 기반 조회, SQLite 데이터베이스 연동
   - TopicManager: 도메인 패턴 기반 토픽 검색, 메시지 타입별 구독 관리
   - PluginManager: 플러그인 동적 로드, 설정 관리 및 저장 기능

2. 패널 시스템 구현:
   - PanelInterface: 모든 패널의 기본 인터페이스 정의
   - BatteryGaugePanel: 로봇 배터리 상태 모니터링 패널 구현
     - 토픽 선택 및 배터리 상태 시각화
     - 배터리 부족 경고 기능
     - 설정 다이아로그를 통한 커스터마이징

3. 외부 라이브러리 통합:
   - QCustomPlot: 실시간 데이터 시각화를 위한 그래프 라이브러리 추가
   - Qt 의존성 확장: Network, Sql, Concurrent, PrintSupport 추가

4. 디렉토리 구조 개선:
   - 패널별 소스 코드 구조화
   - 외부 라이브러리 관리 경로 추가
   - UI 컴포넌트 모듈화 

## 2023-06-11: UI 파일 로드 및 GUI 표시 문제 해결

### 문제 해결 내용
1. 디버그 로그 추가하여 UI 파일 로드 과정 분석
2. `main.cpp` 파일에 디버그 출력 추가
   - 환경 변수 출력 추가
   - Qt 버전 및 디렉토리 정보 출력
   - QUiLoader 경로 정보 추가
3. UI 파일 로드 경로 문제 식별 및 해결
   - AMENT_PREFIX_PATH 환경 변수를 이용한 UI 파일 경로 찾기 기능 확인
   - UI 파일 로드 성공 여부 로깅 기능 추가

### 개선 사항
1. UI 파일 로드 과정에 디버그 메시지 추가하여 문제 진단 용이하게 함
2. 환경 변수를 통한 리소스 검색 로직 강화
3. XCB 윈도우 시스템 연결 확인 및 이벤트 처리 로깅 추가

### 결과
- GUI 애플리케이션이 정상적으로 실행되고 UI가 올바르게 표시됨
- 탭 간 전환, 위젯 표시 등 기본 동작이 정상 작동
- 마우스 이벤트 처리 및 UI 상호작용 정상 동작 확인 