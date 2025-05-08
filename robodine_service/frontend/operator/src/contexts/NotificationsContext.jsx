import React, { createContext, useContext, useState, useEffect } from 'react';
import { useWebSockets } from './WebSocketContext';
import { useAuth } from './AuthContext';

// 알림의 초기 상태 설정
const initialNotificationsState = {
  items: [],
  unreadCount: 0,
  settings: {
    INFO: true,
    WARNING: true,
    ERROR: true,
    DEBUG: false
  }
};

// NotificationsContext 생성 - 기본값 설정
const NotificationsContext = createContext({
  notifications: [],
  unreadCount: 0,
  settings: initialNotificationsState.settings,
  isLoading: false,
  error: null,
  markAsRead: () => {},
  markAllAsRead: () => {},
  updateSettings: () => {},
  requestNotificationPermission: () => Promise.resolve(false)
});

// NotificationsProvider 컴포넌트
export const NotificationsProvider = ({ children }) => {
  const [notifications, setNotifications] = useState(initialNotificationsState);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const { data: wsData, errors: wsErrors } = useWebSockets();
  const { apiCall } = useAuth();

  // 알림 설정 가져오기
  useEffect(() => {
    const fetchSettings = async () => {
      setIsLoading(true);
      try {
        const data = await apiCall('/api/settings');
        
        if (data && data.alert_settings) {
          setNotifications(prev => ({
            ...prev,
            settings: {
              INFO: data.alert_settings.INFO ?? true,
              WARNING: data.alert_settings.WARNING ?? true,
              ERROR: data.alert_settings.ERROR ?? true,
              DEBUG: data.alert_settings.DEBUG ?? false
            }
          }));
        }
      } catch (err) {
        console.error('알림 설정을 가져오는데 실패했습니다:', err);
        setError('알림 설정을 가져오는데 실패했습니다.');
      } finally {
        setIsLoading(false);
      }
    };

    fetchSettings();
  }, [apiCall]);

  // systemlogs 데이터가 변경될 때마다 알림 필터링 및 업데이트
  useEffect(() => {
    if (!wsData?.systemlogs || wsData.systemlogs.length === 0) return;

    // 마지막으로 처리된 로그 ID 확인
    const lastProcessedId = localStorage.getItem('lastProcessedLogId');
    const lastProcessedTimestamp = localStorage.getItem('lastProcessedLogTimestamp');
    
    // 새 로그 필터링
    const newLogs = wsData.systemlogs.filter(log => {
      // ID로 필터링
      if (lastProcessedId && log.id <= parseInt(lastProcessedId)) return false;
      
      // 타임스탬프로 필터링 (보조 체크)
      if (!lastProcessedId && lastProcessedTimestamp && new Date(log.timestamp) <= new Date(lastProcessedTimestamp)) return false;
      
      // 설정에 따라 로그 레벨 필터링
      return notifications.settings[log.level];
    });

    if (newLogs.length > 0) {
      try {
        // 가장 높은 ID와 최신 타임스탬프 찾기
        const highestId = Math.max(...wsData.systemlogs.map(log => log.id || 0));
        const timestamps = wsData.systemlogs.map(log => new Date(log.timestamp).getTime()).filter(t => !isNaN(t));
        const latestTimestamp = timestamps.length > 0 ? new Date(Math.max(...timestamps)) : new Date();
        
        // 로컬 스토리지 업데이트
        localStorage.setItem('lastProcessedLogId', highestId.toString());
        localStorage.setItem('lastProcessedLogTimestamp', latestTimestamp.toISOString());
        
        // 새 알림 추가
        const newNotifications = newLogs.map(log => ({
          id: `log-${log.id || Date.now()}`,
          message: log.message || '새로운 시스템 로그',
          type: log.level || 'INFO',
          time: new Date(log.timestamp || Date.now()),
          read: false,
          source: 'systemlog',
          sourceId: log.id || 0
        }));
        
        setNotifications(prev => ({
          ...prev,
          items: [...newNotifications, ...prev.items].slice(0, 50), // 최대 50개 알림 유지
          unreadCount: prev.unreadCount + newNotifications.length
        }));
        
        // 브라우저 알림 표시
        if (Notification.permission === "granted") {
          newNotifications.forEach(notification => {
            new Notification("로보다인 시스템 알림", {
              body: notification.message,
              icon: "/favicon.ico"
            });
          });
        }
      } catch (err) {
        console.error('알림 처리 중 오류 발생:', err);
      }
    }
  }, [wsData?.systemlogs, notifications.settings]);

  // 알림 읽음 표시
  const markAsRead = (notificationId) => {
    setNotifications(prev => {
      const updatedItems = prev.items.map(item => 
        item.id === notificationId ? { ...item, read: true } : item
      );
      
      // 읽지 않은 알림 재계산
      const unreadCount = updatedItems.filter(item => !item.read).length;
      
      return {
        ...prev,
        items: updatedItems,
        unreadCount
      };
    });
  };

  // 모든 알림 읽음 표시
  const markAllAsRead = () => {
    setNotifications(prev => ({
      ...prev,
      items: prev.items.map(item => ({ ...item, read: true })),
      unreadCount: 0
    }));
  };

  // 알림 설정 업데이트
  const updateSettings = async (newSettings) => {
    setNotifications(prev => ({
      ...prev,
      settings: {
        ...prev.settings,
        ...newSettings
      }
    }));
  };

  // 브라우저 알림 권한 요청
  const requestNotificationPermission = async () => {
    if (!("Notification" in window)) {
      console.log("이 브라우저는 알림을 지원하지 않습니다.");
      return false;
    }
    
    if (Notification.permission !== "granted") {
      const permission = await Notification.requestPermission();
      return permission === "granted";
    }
    
    return true;
  };

  // 컨텍스트 값
  const value = {
    notifications: notifications.items,
    unreadCount: notifications.unreadCount,
    settings: notifications.settings,
    isLoading,
    error,
    markAsRead,
    markAllAsRead,
    updateSettings,
    requestNotificationPermission
  };

  return (
    <NotificationsContext.Provider value={value}>
      {children}
    </NotificationsContext.Provider>
  );
};

// 훅으로 알림 컨텍스트 사용
export const useNotifications = () => {
  return useContext(NotificationsContext);
}; 