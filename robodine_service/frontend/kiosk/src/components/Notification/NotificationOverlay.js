// NotificationOverlay.js
import React, { useState, useEffect } from 'react';

// 단일 알림 배지 컴포넌트
const NotificationBadge = ({ children, type, onClose }) => {
  // 타입에 따른 스타일 및 아이콘 설정
  const getTypeStyles = () => {
    switch (type) {
      case 'success':
        return {
          bg: 'bg-green-100',
          border: 'border-green-400',
          text: 'text-green-800',
          icon: '✅'
        };
      case 'error':
        return {
          bg: 'bg-red-100',
          border: 'border-red-400',
          text: 'text-red-800',
          icon: '❌'
        };
      case 'warning':
        return {
          bg: 'bg-yellow-100',
          border: 'border-yellow-400',
          text: 'text-yellow-800',
          icon: '⚠️'
        };
      case 'info':
      default:
        return {
          bg: 'bg-blue-100',
          border: 'border-blue-400',
          text: 'text-blue-800',
          icon: 'ℹ️'
        };
    }
  };

  const styles = getTypeStyles();

  return (
    <div className={`${styles.bg} ${styles.border} ${styles.text} border rounded-md p-3 mb-2 flex items-center justify-between`}>
      <div className="flex items-center">
        <span className="mr-2">{styles.icon}</span>
        <span>{children}</span>
      </div>
      {onClose && (
        <button 
          className="ml-4 text-gray-500 hover:text-gray-700"
          onClick={onClose}
        >
          &times;
        </button>
      )}
    </div>
  );
};

const NotificationOverlay = ({ notifications = [], onClose }) => {
  // 알림이 없으면 렌더링하지 않음
  if (!notifications || notifications.length === 0) {
    return null;
  }

  return (
    <div className="fixed top-0 right-0 m-4 z-50 w-80">
      {notifications.map((notification, index) => (
        <NotificationBadge 
          key={`${notification.id || index}`}
          type={notification.type || 'info'}
          onClose={() => onClose && onClose(notification.id || index)}
        >
          {notification.message}
        </NotificationBadge>
      ))}
    </div>
  );
};

export default NotificationOverlay; 