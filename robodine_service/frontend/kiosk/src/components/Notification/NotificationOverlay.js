// NotificationOverlay.js
import React, { useEffect, useState } from 'react';

// 알림 배지 컴포넌트
const NotificationBadge = ({ message, className, statusColors = {}, onClose, id }) => {
  // 상태에 따른 스타일 적용
  let statusStyle = '';
  if (message.includes('완료') || message.includes('준비')) {
    statusStyle = statusColors.ready || 'border-2 border-[#48BB78] text-[#48BB78]';
  } else if (message.includes('조리중')) {
    statusStyle = statusColors.cooking || 'border-2 border-[#D69E2E] text-[#D69E2E]';
  }

  // 페이드아웃 효과를 위한 상태
  const [isVisible, setIsVisible] = useState(true);
  const [isFadingOut, setIsFadingOut] = useState(false);

  // 자동 사라짐을 위한 타이머 설정
  useEffect(() => {
    const timer = setTimeout(() => {
      setIsFadingOut(true);
      // 페이드아웃 애니메이션 후 컴포넌트 제거
      setTimeout(() => {
        setIsVisible(false);
        if (onClose) onClose(id);
      }, 500); // 0.5초 애니메이션 후 제거
    }, 2000); // 3초 후 페이드아웃 시작

    return () => clearTimeout(timer); // 컴포넌트가 언마운트되면 타이머 정리
  }, [onClose, id]);

  // 알림이 보이지 않게 됐으면 null 반환
  if (!isVisible) return null;

  // 닫기 버튼 핸들러
  const handleClose = () => {
    setIsFadingOut(true);
    // 애니메이션 후 컴포넌트 제거
    setTimeout(() => {
      setIsVisible(false);
      if (onClose) onClose(id);
    }, 500);
  };

  return (
    <div 
      className={`
        px-4 py-2 rounded-full text-sm font-medium bg-white text-gray-800 shadow mb-2 
        flex items-center justify-between ${statusStyle} ${className || ''} 
        transition-opacity duration-500 ease-in-out
        ${isFadingOut ? 'opacity-0' : 'opacity-100'}
      `}
    >
      <span>{message}</span>
      <button 
        className="ml-4 text-gray-500 hover:text-gray-700 text-lg"
        onClick={handleClose}
        aria-label="알림 닫기"
        tabIndex={0}
      >
        &times;
      </button>
    </div>
  );
};

const NotificationOverlay = ({ notifications = [], onClose }) => {
  // 알림이 없으면 렌더링하지 않음
  if (!notifications || notifications.length === 0) {
    return null;
  }

  return (
    <div className="fixed top-0 left-0 w-full mt-24 z-50 flex flex-col items-center">
      {notifications.map((notification, index) => (
        <NotificationBadge 
          key={`${notification.id || index}`}
          id={notification.id || index}
          message={notification.message}
          statusColors={{
            ready: 'border-2 border-[#48BB78] text-[#48BB78]',
            cooking: 'border-2 border-[#D69E2E] text-[#D69E2E]',
          }}
          onClose={onClose}
        />
      ))}
    </div>
  );
};

export default NotificationOverlay; 