// src/contexts/WebSocketContext.jsx
import React, { createContext, useContext, useState, useEffect, useCallback, useRef } from 'react';

const WS_BASE_URL = 'ws://127.0.0.1:8000/ws';
const TOPICS = ['robots', 'tables', 'events', 'orders', 'status', 'systemlogs', 'customers', 'inventory', 'video_streams'];

// 재연결 관련 상수
const INITIAL_RECONNECT_DELAY = 3000; // 초기 재연결 지연 시간 (3초)
const MAX_RECONNECT_DELAY = 30000;    // 최대 재연결 지연 시간 (30초)
const RECONNECT_DECAY = 1.5;          // 재연결 지연 증가 계수
const MAX_RECONNECT_ATTEMPTS = 5;     // 최대 재연결 시도 횟수

const WebSocketContext = createContext(null);

/**
 * Hook to access WebSocket data and controls.
 */
export function useWebSockets() {
  const context = useContext(WebSocketContext);
  if (!context) {
    throw new Error('useWebSockets must be used within WebSocketProvider');
  }
  return context;
}

/**
 * Provides WebSocket connections & data for all topics.
 */
export function WebSocketProvider({ children }) {
  const [connections, setConnections] = useState({});
  const [data, setData] = useState({
    robots: [], tables: [], events: [], orders: {}, status: {}, systemlogs: [], customers: {}, inventory: [], video_streams: []
  });
  const [errors, setErrors] = useState({});
  const [connected, setConnected] = useState({});
  
  // 재연결 시도 관련 상태 추적
  const reconnectAttempts = useRef({});
  const reconnectTimers = useRef({});
  const backoffDelays = useRef({});

  // WebSocket 상태 디버깅을 위한 로컬 스토리지 로그 기능
  const logToStorage = (topic, event, details) => {
    try {
      // 마지막 5개의 이벤트만 저장
      const logKey = `ws_log_${topic}`;
      const currentLog = JSON.parse(localStorage.getItem(logKey) || '[]');
      const newEntry = {
        timestamp: new Date().toISOString(),
        event,
        details: typeof details === 'string' ? details : JSON.stringify(details)
      };
      
      const updatedLog = [newEntry, ...currentLog].slice(0, 5);
      localStorage.setItem(logKey, JSON.stringify(updatedLog));
    } catch (e) {
      console.debug('로컬 스토리지 로깅 실패:', e);
    }
  };

  // 안전하게 오류 메시지를 추출하는 함수
  const getErrorMessage = (err) => {
    if (!err) return '알 수 없는 오류';
    
    // 이미 문자열인 경우
    if (typeof err === 'string') return err;
    
    // Event 객체인 경우
    if (err instanceof Event) {
      return `연결 오류 (${err.type})`;
    }
    
    // 일반 객체인 경우
    return err.message || err.toString() || '알 수 없는 오류';
  };

  // 다음 재연결 지연 시간 계산
  const calculateNextDelay = (topic) => {
    const currentDelay = backoffDelays.current[topic] || INITIAL_RECONNECT_DELAY;
    const nextDelay = Math.min(currentDelay * RECONNECT_DECAY, MAX_RECONNECT_DELAY);
    backoffDelays.current[topic] = nextDelay;
    return nextDelay;
  };

  // 재연결 타이머 초기화
  const resetReconnection = (topic) => {
    if (reconnectTimers.current[topic]) {
      clearTimeout(reconnectTimers.current[topic]);
      reconnectTimers.current[topic] = null;
    }
    reconnectAttempts.current[topic] = 0;
    backoffDelays.current[topic] = INITIAL_RECONNECT_DELAY;
  };

  // Connect to a single topic (stable callback)
  const connectTopic = useCallback((topic) => {
    try {
      const existing = connections[topic];
      if (existing && (existing.readyState === WebSocket.OPEN || existing.readyState === WebSocket.CONNECTING)) {
        return;
      }
      
      // 이미 최대 재연결 시도를 초과한 경우
      if (reconnectAttempts.current[topic] >= MAX_RECONNECT_ATTEMPTS) {
        console.log(`WebSocket: ${topic} 연결 시도 중단 (최대 시도 횟수 초과)`);
        logToStorage(topic, 'max_attempts_exceeded', { attempts: reconnectAttempts.current[topic] });
        setErrors(prev => ({ ...prev, [topic]: `연결 실패: 최대 시도 횟수(${MAX_RECONNECT_ATTEMPTS}회)를 초과했습니다. 페이지를 새로고침해 주세요.` }));
        return;
      }
      
      // 연결 시도 카운트 증가
      reconnectAttempts.current[topic] = (reconnectAttempts.current[topic] || 0) + 1;
      logToStorage(topic, 'connecting', { attempt: reconnectAttempts.current[topic] });
      
      console.log(`WebSocket: ${topic} 연결 중... (시도 ${reconnectAttempts.current[topic]}/${MAX_RECONNECT_ATTEMPTS})`);
      
      const ws = new WebSocket(`${WS_BASE_URL}/${topic}`);
      
      // 연결 타임아웃 설정 (5초)
      const connectionTimeout = setTimeout(() => {
        if (ws.readyState !== WebSocket.OPEN) {
          console.warn(`WebSocket: ${topic} 연결 타임아웃`);
          logToStorage(topic, 'connection_timeout', null);
          ws.close();
        }
      }, 5000);
      
      ws.onopen = () => {
        console.log(`WebSocket: ${topic} 연결됨`);
        logToStorage(topic, 'connected', null);
        clearTimeout(connectionTimeout);
        setConnected(prev => ({ ...prev, [topic]: true }));
        setErrors(prev => ({ ...prev, [topic]: null }));
        
        // 성공 시 재연결 관련 상태 초기화
        resetReconnection(topic);
      };
      
      ws.onmessage = (evt) => {
        try {
          const msg = JSON.parse(evt.data);
          if (msg.type === 'update' && msg.topic === topic) {
            setData(prev => ({ ...prev, [topic]: msg.data }));
          }
        } catch (err) {
          console.error(`WebSocket: ${topic} 메시지 파싱 오류`, err);
          logToStorage(topic, 'parse_error', err.message);
          setErrors(prev => ({ ...prev, [topic]: getErrorMessage(err) }));
        }
      };
      
      ws.onclose = (event) => {
        clearTimeout(connectionTimeout);
        const isAbnormalClose = event.code !== 1000 && event.code !== 1001;
        
        // 비정상 종료 여부 확인
        if (isAbnormalClose) {
          console.warn(`WebSocket: ${topic} 비정상 종료 (코드: ${event.code}, 이유: ${event.reason || '알 수 없음'})`);
          logToStorage(topic, 'abnormal_close', { code: event.code, reason: event.reason });
        } else {
          console.log(`WebSocket: ${topic} 정상 종료`);
          logToStorage(topic, 'normal_close', { code: event.code });
        }
        
        setConnected(prev => ({ ...prev, [topic]: false }));
        
        // 명시적으로 연결 객체를 정리
        setConnections(prev => {
          const newConn = {...prev};
          delete newConn[topic];
          return newConn;
        });
        
        // 정상 종료가 아닌 경우에만 자동 재연결 시도
        if (isAbnormalClose) {
          // 지수 백오프를 사용한 재연결 지연 계산
          const delay = calculateNextDelay(topic);
          console.log(`WebSocket: ${topic} ${delay}ms 후 재연결 시도 예정`);
          logToStorage(topic, 'reconnect_scheduled', { delay, attempt: reconnectAttempts.current[topic] });
          
          reconnectTimers.current[topic] = setTimeout(() => connectTopic(topic), delay);
        }
      };
      
      ws.onerror = (err) => {
        console.error(`WebSocket: ${topic} 오류 발생`, err);
        logToStorage(topic, 'error', getErrorMessage(err));
        
        // 연결 상태와 타이머 정리
        clearTimeout(connectionTimeout);
        setErrors(prev => ({ ...prev, [topic]: getErrorMessage(err) }));
        
        // 명시적으로 연결 종료
        if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) {
          ws.close();
        }
      };
      
      setConnections(prev => ({ ...prev, [topic]: ws }));
    } catch (err) {
      console.error(`WebSocket: ${topic} 초기화 실패`, err);
      logToStorage(topic, 'initialization_failed', err.message);
      setErrors(prev => ({ ...prev, [topic]: getErrorMessage(err) }));
      
      // 초기화 실패 시에도 재연결 로직 적용
      const delay = calculateNextDelay(topic);
      reconnectTimers.current[topic] = setTimeout(() => connectTopic(topic), delay);
    }
  }, [connections]); // 빈 deps → stable reference

  // Refresh connection for a topic
  const refreshTopic = useCallback((topic) => {
    logToStorage(topic, 'manual_refresh', { status: connected[topic] ? 'connected' : 'disconnected' });
    
    // 이전 연결이 있으면 닫기
    const existing = connections[topic];
    if (existing) {
      try {
        // 연결/연결 중인 상태에서만 명시적으로 닫기
        if (existing.readyState === WebSocket.OPEN || existing.readyState === WebSocket.CONNECTING) {
          existing.close(1000, "Manual refresh requested");
        }
      } catch (err) {
        console.warn(`Error closing WebSocket for ${topic}:`, err);
        logToStorage(topic, 'close_error', err.message);
      }
      
      // 연결 객체 정리
      setConnections(prev => {
        const newConn = {...prev};
        delete newConn[topic];
        return newConn;
      });
    }
    
    // 수동 새로고침 시 재연결 상태 초기화
    resetReconnection(topic);
    
    // 새 연결 시작
    connectTopic(topic);
  }, [connections, connectTopic]);

  // Setup connections once on mount
  useEffect(() => {
    TOPICS.forEach(connectTopic);
    
    return () => {
      // 모든 WebSocket 연결 및 타이머 정리
      Object.values(connections).forEach(ws => {
        try { 
          if (ws && ws.readyState === WebSocket.OPEN) {
            ws.close(1000, "Component unmounting"); 
          }
        } catch (err) {
          console.warn('Error closing WebSocket:', err);
        }
      });
      
      // 모든 재연결 타이머 정리
      Object.values(reconnectTimers.current).forEach(timer => {
        if (timer) clearTimeout(timer);
      });
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return (
    <WebSocketContext.Provider value={{ data, errors, connected, refreshTopic }}>
      {children}
    </WebSocketContext.Provider>
  );
}

// Default export
export default WebSocketProvider;
