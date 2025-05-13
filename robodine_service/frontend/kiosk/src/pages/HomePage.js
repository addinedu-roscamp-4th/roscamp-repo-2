// HomePage.js
import React, { useState, useEffect } from 'react';
import Layout from '../components/Layout/Layout';
import MenuGrid from '../components/Menu/MenuGrid';
import { getMenuItems, filterMenuByCategory } from '../api/menuApi';
import { useCart } from '../context/CartContext';
import NotificationOverlay from '../components/Notification/NotificationOverlay';

const HomePage = () => {
  // 상태 관리
  const [menuItems, setMenuItems] = useState([]);
  const [filteredItems, setFilteredItems] = useState([]);
  const [category, setCategory] = useState('추천 메뉴'); // 기본 카테고리
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [notifications, setNotifications] = useState([]);

  // 장바구니 컨텍스트
  const { addToCart } = useCart();

  // 메뉴 데이터 로드
  useEffect(() => {
    const fetchMenuItems = async () => {
      try {
        setIsLoading(true);
        const data = await getMenuItems();
        setMenuItems(data);
        // 초기 필터링 - 추천 메뉴(샐러드)
        setFilteredItems(filterMenuByCategory(data, category));
        setIsLoading(false);
      } catch (err) {
        setError('메뉴를 불러오는 중 오류가 발생했습니다.');
        setIsLoading(false);
        // 에러 알림 추가
        setNotifications([
          ...notifications,
          { id: Date.now(), type: 'error', message: '메뉴를 불러오는 중 오류가 발생했습니다.' }
        ]);
      }
    };

    fetchMenuItems();
  }, []);

  // 카테고리 변경 시 메뉴 필터링
  useEffect(() => {
    setFilteredItems(filterMenuByCategory(menuItems, category));
  }, [category, menuItems]);

  // 장바구니에 아이템 추가
  const handleAddToCart = (item) => {
    addToCart(item);
    // 성공 알림 추가
    setNotifications([
      ...notifications,
      { id: Date.now(), type: 'success', message: `${item.name}이(가) 장바구니에 추가되었습니다.` }
    ]);

    // 3초 후 알림 자동 제거
    setTimeout(() => {
      setNotifications(prev => prev.filter(n => n.id !== Date.now()));
    }, 3000);
  };

  // 알림 닫기 핸들러
  const handleCloseNotification = (id) => {
    setNotifications(notifications.filter(n => n.id !== id));
  };

  return (
    <Layout category={category} setCategory={setCategory}>
      {/* 알림 오버레이 */}
      <NotificationOverlay 
        notifications={notifications} 
        onClose={handleCloseNotification} 
      />
      
      {/* 로딩 상태 */}
      {isLoading && (
        <div className="flex justify-center items-center h-64">
          <p className="text-gray-500">메뉴를 불러오는 중입니다...</p>
        </div>
      )}
      
      {/* 에러 상태 */}
      {error && !isLoading && (
        <div className="bg-red-100 border border-red-400 text-red-700 p-4 rounded-md">
          <p>{error}</p>
          <button 
            className="mt-2 bg-red-500 text-white px-4 py-2 rounded-md hover:bg-red-600"
            onClick={() => window.location.reload()}
          >
            다시 시도
          </button>
        </div>
      )}
      
      {/* 메뉴 그리드 */}
      {!isLoading && !error && (
        <MenuGrid 
          items={filteredItems} 
          onAddToCart={handleAddToCart} 
        />
      )}
    </Layout>
  );
};

export default HomePage; 