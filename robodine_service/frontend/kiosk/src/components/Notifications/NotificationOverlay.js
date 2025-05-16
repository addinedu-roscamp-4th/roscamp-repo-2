import React, { useState, useEffect } from 'react';

const NotificationItem = ({ id, message, onClose }) => {
  const [isVisible, setIsVisible] = useState(true);
  const [isFadingOut, setIsFadingOut] = useState(false);
  
  // 자동으로 3초 후 사라지기
  useEffect(() => {
    const timer = setTimeout(() => {
      setIsFadingOut(true);
      
      // 애니메이션 완료 후 제거
      setTimeout(() => {
        setIsVisible(false);
        if (onClose) onClose(id);
      }, 500); // 0.5초 페이드아웃
    }, 3000); // 3초 후 페이드아웃 시작
    
    return () => clearTimeout(timer);
  }, [id, onClose]);
  
  // 수동으로 닫기
  const handleClose = () => {
    setIsFadingOut(true);
    
    // 애니메이션 완료 후 제거
    setTimeout(() => {
      setIsVisible(false);
      if (onClose) onClose(id);
    }, 500); // 0.5초 페이드아웃
  };
  
  if (!isVisible) return null;
  
  return (
    <div 
      className={`bg-white border-l-4 border-[#C49E69] shadow-lg rounded-md px-6 py-4 mb-3 relative transition-opacity duration-500 ${
        isFadingOut ? 'opacity-0' : 'opacity-100'
      }`}
    >
      <button 
        onClick={handleClose} 
        className="absolute top-2 right-2 text-gray-400 hover:text-gray-600 text-xl"
        aria-label="닫기"
      >
        ×
      </button>
      <p className="text-gray-800 text-lg font-medium">{message}</p>
    </div>
  );
};

const NotificationOverlay = ({ notifications = [], onClose }) => {
  // 표시할 알림이 없다면 렌더링하지 않음
  if (notifications.length === 0) {
    return null;
  }

  // 중복 ID 방지를 위해 각 알림에 고유 키 할당
  const notificationsWithUniqueKeys = notifications.map((notification, index) => {
    // ID가 없거나 중복될 수 있는 경우 인덱스를 결합하여 고유 ID 생성
    const uniqueId = `${notification.id}-${index}`;
    return {
      ...notification,
      uniqueKey: uniqueId
    };
  });
  
  return (
    <div className="fixed top-4 left-1/2 transform -translate-x-1/2 z-50 w-96">
      {notificationsWithUniqueKeys.map((notification) => (
        <NotificationItem 
          key={notification.uniqueKey} 
          id={notification.id} 
          message={notification.message} 
          onClose={onClose} 
        />
      ))}
    </div>
  );
};

export default NotificationOverlay; 