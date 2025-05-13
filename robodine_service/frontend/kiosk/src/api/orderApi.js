// orderApi.js
import axios from 'axios';

const API_URL = 'http://localhost:8000/api';

// 새 주문 생성
export const createOrder = async (orderData) => {
  try {
    const response = await axios.post(`${API_URL}/orders`, orderData);
    return response.data;
  } catch (error) {
    console.error('주문 생성 중 오류 발생:', error);
    throw error;
  }
};

// 주문 상태 확인
export const getOrderStatus = async (orderId) => {
  try {
    const response = await axios.get(`${API_URL}/orders/${orderId}`);
    return response.data;
  } catch (error) {
    console.error(`주문 ${orderId} 상태 확인 중 오류 발생:`, error);
    throw error;
  }
};

// 새 고객 생성 (테이블 주문 시 필요)
export const createCustomer = async (customerCount) => {
  try {
    const response = await axios.post(`${API_URL}/customers`, {
      count: customerCount
    });
    return response.data;
  } catch (error) {
    console.error('고객 생성 중 오류 발생:', error);
    throw error;
  }
};

// 고객을 테이블에 할당
export const assignCustomerToTable = async (customerId, tableId) => {
  try {
    const response = await axios.put(`${API_URL}/customers/${customerId}/assign-table/${tableId}`);
    return response.data;
  } catch (error) {
    console.error(`고객 ${customerId}을 테이블 ${tableId}에 할당 중 오류 발생:`, error);
    throw error;
  }
};

// 사용 가능한 테이블 목록 가져오기
export const getAvailableTables = async () => {
  try {
    const response = await axios.get(`${API_URL}/tables`);
    // 사용 가능한 테이블만 필터링
    return response.data.filter(table => table.status === 'AVAILABLE');
  } catch (error) {
    console.error('사용 가능한 테이블 조회 중 오류 발생:', error);
    throw error;
  }
}; 