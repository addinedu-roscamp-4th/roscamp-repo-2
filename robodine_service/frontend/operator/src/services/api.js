import { API_BASE_URL, API_ENDPOINTS, getDefaultHeaders } from '../config/api';

/**
 * 기본 API 요청 함수
 * @param {string} url - API 엔드포인트 URL
 * @param {Object} options - fetch 옵션
 * @returns {Promise} - API 응답
 */
const fetchApi = async (url, options = {}) => {
  try {
    const response = await fetch(`${API_BASE_URL}${url}`, options);
    
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.message || `${response.status}: ${response.statusText}`);
    }
    
    return await response.json();
  } catch (error) {
    console.error('API 요청 오류:', error);
    throw error;
  }
};

/**
 * 토큰 가져오기
 * @returns {string|null} - 저장된 토큰
 */
const getToken = () => localStorage.getItem('token');

/**
 * API 서비스 객체
 */
const apiService = {
  // 인증 관련
  auth: {
    login: async (credentials) => {
      return fetchApi(API_ENDPOINTS.LOGIN, {
        method: 'POST',
        headers: getDefaultHeaders(),
        body: JSON.stringify(credentials),
      });
    },
    
    signup: async (userData) => {
      return fetchApi(API_ENDPOINTS.SIGNUP, {
        method: 'POST',
        headers: getDefaultHeaders(),
        body: JSON.stringify(userData),
      });
    },
    
    validateToken: async () => {
      const token = getToken();
      if (!token) return Promise.reject(new Error('No token found'));
      
      return fetchApi(API_ENDPOINTS.VALIDATE_TOKEN, {
        method: 'GET',
        headers: getDefaultHeaders(token),
      });
    },
  },
  
  // 로봇 관련
  robots: {
    getAll: async () => {
      return fetchApi(API_ENDPOINTS.ROBOTS, {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
    
    getById: async (id) => {
      return fetchApi(API_ENDPOINTS.ROBOT_DETAIL(id), {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
    
    sendCommand: async (id, command) => {
      return fetchApi(API_ENDPOINTS.ROBOT_COMMAND(id), {
        method: 'POST',
        headers: getDefaultHeaders(getToken()),
        body: JSON.stringify(command),
      });
    },
  },
  
  // 테이블 관련
  tables: {
    getAll: async () => {
      return fetchApi(API_ENDPOINTS.TABLES, {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
    
    getById: async (id) => {
      return fetchApi(API_ENDPOINTS.TABLE_DETAIL(id), {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
  },
  
  // 주문 관련
  orders: {
    getAll: async () => {
      return fetchApi(API_ENDPOINTS.ORDERS, {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
    
    getById: async (id) => {
      return fetchApi(API_ENDPOINTS.ORDER_DETAIL(id), {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
  },
  
  // 이벤트 관련
  events: {
    getAll: async () => {
      return fetchApi(API_ENDPOINTS.EVENTS, {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
    
    getById: async (id) => {
      return fetchApi(API_ENDPOINTS.EVENT_DETAIL(id), {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
  },
  
  // 시스템 상태
  system: {
    getStatus: async () => {
      return fetchApi(API_ENDPOINTS.SYSTEM_STATUS, {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
  },
  
  // 통계 관련
  stats: {
    getOrderStats: async () => {
      return fetchApi(API_ENDPOINTS.STATS_ORDERS, {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
    
    getInventoryStats: async () => {
      return fetchApi(API_ENDPOINTS.STATS_INVENTORY, {
        method: 'GET',
        headers: getDefaultHeaders(getToken()),
      });
    },
  },
};

export default apiService; 