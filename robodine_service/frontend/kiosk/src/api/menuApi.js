// menuApi.js
import axios from 'axios';

const API_URL = process.env.REACT_APP_BASE_URL

// 모든 메뉴 항목 가져오기
export const getMenuItems = async () => {
  try {
    const response = await axios.get(`${API_URL}/menu/items`);
    return response.data;
  } catch (error) {
    console.error('메뉴를 불러오는 중 오류 발생:', error);
    throw error;
  }
};

// 메뉴 항목 상세 정보 가져오기
export const getMenuItem = async (itemId) => {
  try {
    const response = await axios.get(`${API_URL}/menu/items/${itemId}`);
    return response.data;
  } catch (error) {
    console.error(`메뉴 항목 ${itemId}를 불러오는 중 오류 발생:`, error);
    throw error;
  }
};

// 메뉴 카테고리로 필터링 하는 함수
export const filterMenuByCategory = (menuItems, category) => {
  if (!category || category === '전체') return menuItems;
  
  // 추천 메뉴는 샐러드 필터링
  if (category === '추천 메뉴') {
    return menuItems.filter(item => item.name.includes('샐러드'));
  }
  
  // 음식 / 음료 카테고리에 따라 필터링
  return menuItems.filter(item => {
    // 이름에 '음료', '커피', '주스' 등이 포함되면 음료로 간주
    const isDrink = item.name.includes('음료') || 
                   item.name.includes('커피') ||
                   item.name.includes('주스') ||
                   item.name.includes('차');
                   
    return category === '음료' ? isDrink : !isDrink;
  });
}; 