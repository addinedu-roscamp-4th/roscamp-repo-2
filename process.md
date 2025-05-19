# 프로세스 변경 내역

## 2024-03-21: 키오스크 청소 완료 버튼 기능 추가

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