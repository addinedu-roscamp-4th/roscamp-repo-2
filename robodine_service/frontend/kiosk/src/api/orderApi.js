// orderApi.js
import axios from 'axios';

const API_URL = process.env.REACT_APP_BASE_URL
const KIOSK_ID = 1; // 예시로 KIOSK_ID를 1로 설정, 실제 값으로 변경 필요

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

// 키오스크가 속한 테이블 정보 가져오기
export const getKioskTableInfo = async (kioskId) => {
  console.log('키오스크 ID:', kioskId);
  try {
    const response = await axios.get(`${API_URL}/kiosks`);
    const kiosks = response.data;
    const kiosk = kiosks.find(k => k.id === kioskId);
    if (!kiosk) {
      throw new Error(`키오스크 ${kioskId}를 찾을 수 없습니다.`);
    }
    const tableId = kiosk.table_id;
    const tableResponse = await axios.get(`${API_URL}/tables`);
    const tables = tableResponse.data;
    const table = tables.find(t => t.id === tableId);
    if (!table) {
      throw new Error(`테이블 ${tableId}를 찾을 수 없습니다.`);
    }
    return {
      tableId: table.id,
      status: table.status,
      max_customer: table.max_customer
    };
  } catch (error) {
    console.error('키오스크 테이블 정보 조회 중 오류 발생:', error);
    throw error;
  }
}

// 고객이 속한 그룹 정보 조회
export const getCustomerGroupInfo = async (table_id) => {
  try {
    const response = await axios.get(`${API_URL}/tables/assignments`);
    const tables = response.data;
    const table = tables.find(t => t.table_id === table_id);
    if (!table) {
      throw new Error(`테이블 ${table_id}를 찾을 수 없습니다.`);
    }
    return {
      customerId: table.customer_id
    };
  } catch (error) {
    console.error('고객 그룹 정보 조회 중 오류 발생:', error);
    throw error;
  }
}
    

// 고객을 테이블에 할당
export const assignCustomerToTable = async (customerId, tableId) => {
  try {
    const response = await axios.post(`${API_URL}/tables/${tableId}/assign`, {
      customer_id: customerId
    });
    return response.data;
  } catch (error) {
    console.error(`고객 ${customerId}을 테이블 ${tableId}에 할당 중 오류 발생:`, error);
    throw error;
  }
};

// 새 고객 생성 (테이블 주문 시 필요)
export async function createCustomer(count) {
  try {
    // 1) 키오스크가 속한 테이블 정보 조회
    const tableInfo = await getKioskTableInfo(KIOSK_ID);
    const { tableId, status } = tableInfo;

    let customerId;

    if (status !== 'AVAILABLE') {
      // 2a) 사용 중인 테이블이면 기존 고객 ID 조회
      const { customerId: existingId } = await getCustomerGroupInfo(tableId);
      customerId = existingId;
    } else {
      // 2b) 사용 가능 테이블이면 새 고객 생성
      const response = await axios.post(`${API_URL}/customers`, { count });
      console.log('새 고객 생성 응답:', response.data);
      customerId = response.data.id;

      // 3) 새로 생성된 고객을 테이블에 할당
      await assignCustomerToTable(customerId, tableId);
    }

    // 4) 최종 결과 반환
    return { customer_id: customerId, tableId };
  } catch (error) {
    console.error('createCustomer error:', error);
    throw error;
  }
}


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