# RoboDine 운영자 대시보드

RoboDine 운영자 대시보드는 레스토랑 로봇 관리 시스템의 관리자용 웹 인터페이스입니다. 이 대시보드를 통해 로봇 상태 모니터링, 주문 관리, 시스템 이벤트 추적 등을 할 수 있습니다.

## 목차

- [설치 및 설정](#설치-및-설정)
- [실행 방법](#실행-방법)
- [폴더 구조](#폴더-구조)
- [페이지 구성](#페이지-구성)
- [데이터 흐름](#데이터-흐름)
- [컴포넌트 수정 가이드](#컴포넌트-수정-가이드)
- [API 엔드포인트](#api-엔드포인트)

## 설치 및 설정

### 요구 사항

- Node.js (v14+)
- npm 또는 yarn

### 설치 과정

1. 저장소 클론하기:
```bash
git clone [repository-url]
cd roscamp-repo-2/robodine_service/frontend/operator
```

2. 의존성 설치:
```bash
npm install
# 또는
yarn install
```

## 실행 방법

### 개발 환경

개발 모드로 애플리케이션을 실행하려면:

```bash
npm start
# 또는
yarn start
```

개발 서버가 시작되며 기본적으로 `http://localhost:3000`에서 접근할 수 있습니다.

### 프로덕션 빌드

프로덕션용 빌드를 생성하려면:

```bash
npm run build
# 또는
yarn build
```

빌드 후, 생성된 정적 파일은 `build` 디렉토리에 저장됩니다. 이 파일들을 원하는 웹 서버에 배포할 수 있습니다.

## 폴더 구조

```
operator/
├── public/             # 정적 파일
├── src/                # 소스 코드
│   ├── components/     # 재사용 가능한 컴포넌트
│   │   ├── dashboard/  # 대시보드 관련 컴포넌트
│   │   └── Layout.jsx  # 레이아웃 컴포넌트
│   ├── contexts/       # React 컨텍스트 (상태 관리)
│   ├── pages/          # 페이지 컴포넌트
│   ├── App.js          # 애플리케이션의 진입점
│   ├── index.js        # React DOM 렌더링
│   ├── index.css       # 전역 스타일
│   └── mockData.js     # 개발용 더미 데이터
├── package.json        # 프로젝트 의존성 및 스크립트
├── tailwind.config.js  # Tailwind CSS 설정
└── README.md           # 프로젝트 문서
```

## 페이지 구성

대시보드는 다음과 같은 페이지로 구성됩니다:

### 1. 대시보드 (DashboardPage)
- 로봇 상태 패널: 모든 로봇의 현재 상태를 표시합니다.
- 매장 맵: 테이블과 로봇의 실시간 위치를 시각화합니다.
- 이벤트 타임라인: 시스템 이벤트를 시간순으로 표시합니다.
- 최근 주문: 최근 주문 내역을 표시하고 주문 상태를 색상으로 구분합니다.

### 2. 영상 스트리밍 (VideoStreamPage)
- 로봇 및 매장 카메라의 실시간 영상 스트림을 제공합니다.
- 글로벌 카메라, 요리 로봇, 서빙 로봇 등 다양한 소스별 영상을 보여줍니다.

### 3. 로봇 관리 (RobotAdminPage)
- 로봇 목록 조회 및 관리
- 로봇 상태 변경 및 명령 전송
- 로봇 상세 정보 및 명령 이력 조회

### 4. 고객/테이블 관리 (CustomerPage)
- 테이블 상태 모니터링
- 고객 정보 관리
- 테이블 할당 및 해제

### 5. 주문/재고 통계 (StatsPage)
- 판매 통계 및 차트
- 재고 현황 및 경고 알림
- 기간별 매출 보고서

### 6. 이벤트/로그 (SystemPage)
- 시스템 이벤트 로그
- 오류 및 경고 메시지
- 로그 데이터 내보내기

### 7. 설정 (SettingsPage)
- 시스템 운영 시간 설정
- 재고 경고 임계값 설정
- 알림 설정 관리

## 대시보드의 주요 컴포넌트

### RecentOrders
- 주문 ID, 테이블 번호, 주문 시간, 주문 항목 수, 총 금액, 주문 상태를 테이블 형식으로 표시
- 주문 상태별로 다른 아이콘과 색상으로 시각화(완료, 취소, 대기 중, 처리 중)
- 각 주문에 대한 상세 보기 버튼 제공
- 한국어 날짜/시간 형식 및 원화 표시 지원

### SystemStatus
- 서버, 로봇, 배터리, 네트워크 등 다양한 시스템 구성 요소의 상태 표시
- 각 시스템 유형별 적절한 아이콘으로 시각화
- 상태(온라인, 경고, 오프라인)에 따른 색상 구분
- 마지막 업데이트 시간 표시

### EventTimeline
- 시스템 이벤트를 시간순으로 정렬하여 표시
- 이벤트 유형(경고, 오류, 시스템, 로봇, 비상)별 필터링 기능
- 타임라인 형식의 시각적 표현
- 이벤트 선택 시 맵에 해당 위치 하이라이트 기능

### RobotStatusPanel
- 로봇 상태, 배터리 수준, IP 주소, 최근 활동 시간 등 표시
- 로봇 유형별 아이콘 제공
- 상태별 색상 코드로 시각화
- 상세 정보 모달을 통한 추가 정보 제공

### StoreMap
- 테이블과 로봇의 실시간 위치 시각화
- 줌 인/아웃 및 리셋 기능
- 테이블 상태(사용 가능/사용 중)에 따른 색상 구분
- 선택된 이벤트 발생 위치 하이라이트

## 데이터 흐름

### 컴포넌트 구조

```
App
├── Router (BrowserRouter)
│   └── Routes
│       ├── DashboardPage
│       │   ├── Layout
│       │   │   ├── Sidebar
│       │   │   └── Header
│       │   ├── RobotStatusPanel
│       │   ├── StoreMap
│       │   ├── EventTimeline
│       │   ├── RecentOrders
│       │   └── SystemStatus
│       ├── VideoStreamPage
│       ├── RobotAdminPage
│       ├── CustomerPage
│       ├── StatsPage
│       ├── SystemPage
│       └── SettingsPage
└── Contexts (추가 예정)
```

### 데이터 관리

1. **API 통신**
   - 각 페이지에서 `fetch` API를 사용하여 백엔드 서버와 통신합니다.
   - 주요 API 경로는 `/api/` 접두사로 시작합니다.
   - 개발 환경에서는 `mockData.js`의 더미 데이터를 사용합니다.

2. **실시간 업데이트**
   - `useEffect`와 `setInterval`을 사용하여 주기적으로 데이터를 폴링합니다.
   - 대시보드의 경우 30초마다 데이터를 자동으로 새로고침합니다.

3. **데이터 흐름 예시: 대시보드**
   - `DashboardPage` 컴포넌트에서 데이터를 불러와 하위 컴포넌트에 전달
   - 각 하위 컴포넌트(RobotStatusPanel, StoreMap, EventTimeline 등)는 전달받은 데이터를 표시
   - 사용자 상호작용(예: 이벤트 선택)은 콜백을 통해 상위 컴포넌트로 전달

## 컴포넌트 수정 가이드

### 대시보드 컴포넌트 수정

1. **EventTimeline 컴포넌트** (`src/components/dashboard/EventTimeline.jsx`)
   - 이벤트 필터링 로직: `filter` 상태와 `filteredEvents` 변수를 수정하여 이벤트 필터링 방식을 변경할 수 있습니다.
   - 이벤트 유형별 아이콘/색상: `getEventIcon` 및 `getEventColor` 함수를 통해 이벤트 시각화를 변경할 수 있습니다.

2. **RecentOrders 컴포넌트** (`src/components/dashboard/RecentOrders.jsx`)
   - 주문 상태별 스타일: `getStatusIcon` 및 `getStatusClass` 함수를 통해 주문 상태 시각화를 변경할 수 있습니다.
   - 날짜/시간 형식: `formatTimestamp` 함수를 수정하여 시간 표시 형식을 변경할 수 있습니다.
   - 가격 형식: `formatPrice` 함수를 통해 가격 표시 형식을 변경할 수 있습니다.
   - 테이블 컬럼 구성: 테이블 헤더와 각 행의 내용을 수정하여 표시되는 정보를 변경할 수 있습니다.

3. **SystemStatus 컴포넌트** (`src/components/dashboard/SystemStatus.jsx`)
   - 시스템 유형별 아이콘: `getSystemIcon` 함수를 통해 시스템 유형별 아이콘을 변경할 수 있습니다.
   - 상태별 스타일: `getStatusIcon` 및 `getStatusClass` 함수를 통해 상태 표시 스타일을 변경할 수 있습니다.
   - 시스템 정보 표시: 각 시스템 항목의 표시 형식을 수정하여 추가 정보를 표시할 수 있습니다.

4. **RobotStatusPanel 컴포넌트** (`src/components/dashboard/RobotStatusPanel.jsx`)
   - 로봇 유형별 아이콘: `getRobotIcon` 함수를 수정하여 로봇 유형별 아이콘을 변경할 수 있습니다.
   - 상태별 색상: `getStatusColor` 함수를 통해 로봇 상태별 색상을 변경할 수 있습니다.

5. **StoreMap 컴포넌트** (`src/components/dashboard/StoreMap.jsx`)
   - 테이블 위치: 테이블과 로봇의 위치 정보를 수정하여 맵에서의 표시 위치를 조정할 수 있습니다.
   - 로봇 및 테이블 색상: `getRobotColor` 및 `getTableColor` 함수를 통해 색상을 변경할 수 있습니다.

### 새 컴포넌트 추가

새로운 대시보드 컴포넌트를 추가하려면:

1. `src/components/dashboard` 디렉토리에 새 컴포넌트 파일을 생성합니다.
2. 컴포넌트를 구현합니다.
3. `src/pages/DashboardPage.jsx`에서 import하고 렌더링 부분에 추가합니다.

예:
```jsx
// 새 컴포넌트 import
import NewComponent from '../components/dashboard/NewComponent';

// 렌더링 부분에 추가
<div className="grid grid-cols-1 lg:grid-cols-3 gap-4 p-4">
  <div className="lg:col-span-2">
    <div className="grid grid-cols-1 gap-4">
      <RobotStatusPanel robots={robots} />
      <RecentOrders orders={orders} onViewOrder={handleViewOrder} />
      <NewComponent data={someData} />
    </div>
  </div>
  {/* ... */}
</div>
```

### 페이지 레이아웃 수정

레이아웃을 수정하려면 `src/components/Layout.jsx` 파일을 수정하세요:

- **네비게이션 메뉴**: `navItems` 배열을 변경하여 사이드바 메뉴를 수정할 수 있습니다.
- **헤더 내용**: 헤더 부분을 수정하여 로고, 사용자 정보, 추가 버튼 등을 변경할 수 있습니다.
- **레이아웃 스타일**: 클래스를 수정하여 전체 레이아웃 스타일을 변경할 수 있습니다.

## API 엔드포인트

대시보드는 다음과 같은 API 엔드포인트와 통신합니다:

### 대시보드 데이터
- `GET /api/robots`: 로봇 정보 조회
- `GET /api/tables`: 테이블 정보 조회
- `GET /api/events`: 이벤트 데이터 조회
- `GET /api/robot/:id/position`: 로봇 위치 조회
- `GET /api/orders`: 주문 정보 조회
- `GET /api/systems`: 시스템 상태 정보 조회

### 로봇 관리
- `GET /api/robot/commands/:id`: 로봇 명령 이력 조회
- `POST /api/robot/commands/:id/command`: 로봇에 명령 전송

### 영상 스트림
- `GET /api/video-streams`: 영상 스트림 목록 조회
- `PUT /api/video-streams/:id/refresh`: 스트림 새로고침

### 설정
- `GET /api/settings`: 시스템 설정 조회
- `PUT /api/settings`: 시스템 설정 업데이트

## 개발 환경에서의 더미 데이터

개발 환경에서는 백엔드 API가 없어도 애플리케이션을 테스트할 수 있도록 더미 데이터를 제공합니다. 더미 데이터는 `src/mockData.js` 파일에 정의되어 있으며, 필요에 따라 수정할 수 있습니다.

실제 백엔드 API와 연동하려면 `src/App.js` 파일에서 mock API 설정 부분을 제거하거나 수정하세요:

```jsx
// 이 부분을 제거하거나 실제 백엔드 URL로 변경
if (process.env.NODE_ENV === 'development') {
  window.fetch = (url) => {
    // mock API 구현...
  };
}
```

## 추가 개발 시 참고사항

1. **새로운 페이지 추가**
   - `src/pages` 디렉토리에 페이지 컴포넌트를 생성합니다.
   - `src/App.js`의 Route 설정에 해당 페이지를 추가합니다.

2. **스타일링**
   - 애플리케이션은 Tailwind CSS를 사용하여 스타일링됩니다.
   - 커스텀 스타일은 CSS 모듈이나 Tailwind 설정을 통해 추가할 수 있습니다.

3. **컨텍스트 추가**
   - 전역 상태 관리가 필요한 경우 `src/contexts` 디렉토리에 새 컨텍스트를 생성할 수 있습니다.
   - React Context API 또는 Redux 등을 사용하여 상태 관리를 구현할 수 있습니다.

## 문제 해결

- **"API 응답 없음" 오류**: 개발 환경에서는 기본적으로 더미 데이터를 사용합니다. 이 오류가 발생한다면 더미 데이터 설정을 확인하세요.
- **컴포넌트 렌더링 문제**: React DevTools를 사용하여 컴포넌트 트리와 props를 검사하세요.
- **스타일 문제**: 브라우저 개발자 도구를 사용하여 적용된 스타일을 검사하세요. 