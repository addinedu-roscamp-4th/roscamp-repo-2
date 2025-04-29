// API 기본 주소
export const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || '';

// API 엔드포인트
export const API_ENDPOINTS = {
  // 인증
  LOGIN: '/api/auth/login',
  SIGNUP: '/api/auth/signup',
  VALIDATE_TOKEN: '/api/auth/validate',
  
  // 로봇 관련
  ROBOTS: '/api/robots',
  ROBOT_DETAIL: (id) => `/api/robots/${id}`,
  ROBOT_COMMAND: (id) => `/api/robots/${id}/command`,
  
  // 테이블 관련
  TABLES: '/api/tables',
  TABLE_DETAIL: (id) => `/api/tables/${id}`,
  
  // 주문 관련
  ORDERS: '/api/orders',
  ORDER_DETAIL: (id) => `/api/orders/${id}`,
  
  // 이벤트 관련
  EVENTS: '/api/events',
  EVENT_DETAIL: (id) => `/api/events/${id}`,
  
  // 시스템 상태
  SYSTEM_STATUS: '/api/system/status',
  
  // 통계 관련
  STATS_ORDERS: '/api/stats/orders',
  STATS_INVENTORY: '/api/stats/inventory',
};

// API 호출 기본 헤더
export const getDefaultHeaders = (token) => {
  const headers = {
    'Content-Type': 'application/json',
  };
  
  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }
  
  return headers;
}; 