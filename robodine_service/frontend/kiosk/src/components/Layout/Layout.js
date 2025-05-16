import React, { useState, useEffect } from 'react';
import { Outlet, useLocation } from 'react-router-dom';
import Sidebar from './Sidebar';
import NotificationOverlay from '../Notifications/NotificationOverlay';

const Layout = () => {
  const [selectedCategory, setSelectedCategory] = useState('추천');
  const [notifications, setNotifications] = useState([]);
  const location = useLocation();

  // 페이지 변경 시 알림 초기화
  useEffect(() => {
    setNotifications([]);
  }, [location.pathname]);

  // 알림 닫기 핸들러
  const handleCloseNotification = (id) => {
    setNotifications(prev => prev.filter(notification => notification.id !== id));
  };
  
  return (
    <div className="flex h-screen bg-gray-100">
      <Sidebar 
        selectedCategory={selectedCategory} 
        onSelectCategory={setSelectedCategory} 
      />
      
      <main className="flex-grow flex flex-col relative">
        {/* 알림 오버레이 */}
        <NotificationOverlay 
          notifications={notifications} 
          onClose={handleCloseNotification} 
        />
        
        {/* 메인 컨텐츠 */}
        <Outlet context={{ 
          selectedCategory, 
          onSelectCategory: setSelectedCategory,
          setNotifications
        }} />
      </main>
    </div>
  );
};

export default Layout; 