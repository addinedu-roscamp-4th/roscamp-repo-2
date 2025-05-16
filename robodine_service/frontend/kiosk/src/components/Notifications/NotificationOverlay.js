import React, { useEffect, useState } from 'react';

const Notification = ({ message, id, onClose, timeout = 2000 }) => {
  const [isClosing, setIsClosing] = useState(false);

  useEffect(() => {
    // 알림이 일정 시간이 지난 후 자동으로 사라지도록 타이머 설정
    const timer = setTimeout(() => {
      setIsClosing(true);
      setTimeout(() => {
        onClose(id);
      }, 300); // 페이드아웃 애니메이션 시간
    }, timeout);

    return () => {
      clearTimeout(timer);
    };
  }, [id, onClose, timeout]);

  return (
    <div
      className={`bg-white rounded-lg shadow-lg border-l-4 border-[#C49E69] p-6 mb-4 transition-all duration-300 transform ${
        isClosing ? 'opacity-0 translate-y-12' : 'opacity-100 translate-y-0'
      }`}
      style={{ maxWidth: '400px' }}
    >
      <div className="flex justify-between items-center">
        <p className="text-xl font-semibold">{message}</p>
        <button
          onClick={() => {
            setIsClosing(true);
            setTimeout(() => {
              onClose(id);
            }, 300);
          }}
          className="text-gray-400 hover:text-gray-600 focus:outline-none"
        >
          <span className="text-2xl">×</span>
        </button>
      </div>
    </div>
  );
};

const NotificationOverlay = ({ notifications, onClose }) => {
  return (
    <div className="fixed top-4 left-1/2 transform -translate-x-1/2 z-50 pointer-events-none">
      <div className="flex flex-col items-center pointer-events-auto space-y-4">
        {notifications.map((notification) => (
          <Notification
            key={notification.id}
            id={notification.id}
            message={notification.message}
            onClose={onClose}
          />
        ))}
      </div>
    </div>
  );
};

export default NotificationOverlay;
