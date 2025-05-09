import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import { useWebSockets } from './WebSocketContext';
import { useAuth } from './AuthContext';

// 로깅 함수
const logDebug = (...args) => {
  // console.log('[알림]', ...args);
};

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
  const { data: wsData, errors: wsErrors, connected, requestNotificationPermission } = useWebSockets();
  const { apiCall } = useAuth();
  
  // 웹소켓 데이터 로깅
  useEffect(() => {
    if (wsData) {
      logDebug('웹소켓 데이터 변경됨:', { 
        systemlogs: !!wsData.systemlogs,
        systemlogsLength: wsData.systemlogs && Array.isArray(wsData.systemlogs) ? wsData.systemlogs.length : 0,
        notifications: !!wsData.notifications,
        notificationsLength: wsData.notifications && Array.isArray(wsData.notifications) ? wsData.notifications.length : 0,
        connected
      });
    }
    
    if (wsErrors) {
      logDebug('웹소켓 오류:', wsErrors);
    }
  }, [wsData, wsErrors, connected]);

  // 알림 설정 가져오기
  useEffect(() => {
    const fetchSettings = async () => {
      setIsLoading(true);
      try {
        logDebug('알림 설정 가져오기 시도');
        const data = await apiCall('/api/settings');
        logDebug('알림 설정 응답:', data);
        
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
          logDebug('알림 설정 업데이트 완료');
        } else {
          logDebug('알림 설정이 응답에 없거나 형식이 맞지 않음');
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

  // 웹소켓에서 받은 알림 데이터 처리
  useEffect(() => {
    if (wsData && wsData.notifications && Array.isArray(wsData.notifications)) {
      logDebug(`웹소켓에서 ${wsData.notifications.length}개의 알림 수신`);
      
      // 새 알림을 items에 추가 (이미 읽은 항목은 제외)
      setNotifications(prev => {
        // 기존 알림 ID 목록 생성
        const existingIds = new Set(prev.items.map(item => item.id));
        
        // 새 알림 필터링 (이미 있는 알림 제외)
        const newNotifications = wsData.notifications.filter(notif => 
          !existingIds.has(`notif-${notif.id}`)
        ).map(notif => ({
          id: `notif-${notif.id}`,
          message: notif.message || '새로운 알림',
          type: notif.type || 'INFO',
          time: notif.created_at ? new Date(notif.created_at) : new Date(),
          read: false,
          source: 'notification',
          sourceId: notif.id,
          userId: notif.user_id
        }));
        
        if (newNotifications.length === 0) {
          return prev; // 새 알림이 없으면 상태 업데이트 불필요
        }
        
        const updatedItems = [...newNotifications, ...prev.items].slice(0, 50);
        const updatedUnreadCount = updatedItems.filter(item => !item.read).length;
        
        logDebug(`알림 상태 업데이트 (총 ${updatedItems.length}개, 읽지 않음 ${updatedUnreadCount}개)`);
        
        return {
          ...prev,
          items: updatedItems,
          unreadCount: updatedUnreadCount
        };
      });
    }
  }, [wsData.notifications]);

  // 알림 읽음 표시 (백엔드에도 상태 업데이트)
  const markAsRead = useCallback(async (notificationId) => {
    logDebug(`알림 읽음 표시: ${notificationId}`);
    
    // 로컬 상태 업데이트
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
    
    // 서버에 읽음 상태 업데이트 (notification 소스인 경우)
    const notification = notifications.items.find(item => item.id === notificationId);
    if (notification && notification.source === 'notification' && notification.sourceId) {
      try {
        // API 호출을 통해 알림 상태 업데이트
        // await apiCall(`/api/customers/${currentCustomer.id}`, 'PUT', { count: currentCustomer.count });

        await apiCall(`/api/users/${notification.userId}/notifications/${notification.sourceId}`,'PUT',{ status: 'SENT' }
        );
        logDebug(`알림 ${notification.sourceId} 상태 업데이트 완료`);
      } catch (err) {
        console.error('알림 상태 업데이트 실패:', err);
      }
    }
  }, [apiCall, notifications.items]);

  // 모든 알림 읽음 표시 (백엔드에도 상태 업데이트)
  const markAllAsRead = useCallback(async () => {
    logDebug('모든 알림 읽음 표시');
    
    // 읽지 않은 notification 소스 알림 ID 수집
    const unreadNotificationIds = notifications.items
      .filter(item => !item.read && item.source === 'notification' && item.sourceId)
      .map(item => ({
        id: item.sourceId,
        userId: item.userId
      }));
    
    // 로컬 상태 업데이트
    setNotifications(prev => ({
      ...prev,
      items: prev.items.map(item => ({ ...item, read: true })),
      unreadCount: 0
    }));
    
    // 서버에 모든 알림 상태 업데이트
    if (unreadNotificationIds.length > 0) {
      try {
        // 각 알림에 대해 상태 업데이트 API 호출
        // await apiCall(`/api/users/${notification.userId}/notifications/${notification.sourceId}`,'PUT',{ status: 'SENT' }

        const updatePromises = unreadNotificationIds.map(({ id, userId }) => 
          apiCall(`/api/users/${userId}/notifications/${id}`,'PUT',{ status: 'SENT' }
          )
        );
        
        await Promise.all(updatePromises);
        logDebug(`${unreadNotificationIds.length}개 알림 상태 일괄 업데이트 완료`);
      } catch (err) {
        console.error('일괄 알림 상태 업데이트 실패:', err);
      }
    }
  }, [apiCall, notifications.items]);

  // 알림 설정 업데이트
  const updateSettings = useCallback(async (newSettings) => {
    logDebug('알림 설정 업데이트:', newSettings);
    
    // 로컬 상태 업데이트
    setNotifications(prev => ({
      ...prev,
      settings: {
        ...prev.settings,
        ...newSettings
      }
    }));
    
    // 서버에 설정 업데이트
    try {
      await apiCall('/api/settings', {
        method: 'PUT',
        data: {
          alert_settings: {
            ...notifications.settings,
            ...newSettings
          }
        }
      });
      logDebug('알림 설정 서버 업데이트 완료');
    } catch (err) {
      console.error('알림 설정 업데이트 실패:', err);
      setError('알림 설정 업데이트 실패');
    }
  }, [apiCall, notifications.settings]);

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

  // 데이터 로깅
  useEffect(() => {
    logDebug('현재 알림 상태:', { 
      count: notifications.items.length, 
      unread: notifications.unreadCount,
      settings: notifications.settings
    });
  }, [notifications.items.length, notifications.unreadCount]);

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