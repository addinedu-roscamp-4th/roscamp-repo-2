import React, { createContext, useContext, useEffect, useState, useCallback, useRef, useMemo } from 'react';

// WebSocket endpoint
const rawBaseUrl = process.env.REACT_APP_WS_URL || 'ws://192.168.0.156:8000/ws';
const WS_BASE_URL = rawBaseUrl.replace(/\/+$/, '');  // remove trailing slash
const WSContext = createContext(null);

export function UnifiedWebSocketProvider({ topics = ['orders', 'menu', 'tables'], children }) {
  // Memoize topics so effect runs once
  const stableTopics = useMemo(() => topics, []);

  // Track WS connections without causing re-render
  const connectionsRef = useRef({});
  const isConnectingRef = useRef({});

  // React state
  const [data, setData] = useState(() => Object.fromEntries(stableTopics.map(t => [t, null])));
  const [connected, setConnected] = useState(() => Object.fromEntries(stableTopics.map(t => [t, false])));
  const [errors, setErrors] = useState(() => Object.fromEntries(stableTopics.map(t => [t, null])));
  const [currentCustomer, setCurrentCustomer] = useState(null);
  const [customerOrders, setCustomerOrders] = useState([]);

  // Reconnection state
  const reconnectAttempts = useRef({});
  const reconnectTimers = useRef({});
  const backoffDelays = useRef({});
  const isReconnecting = useRef(false);

  // Backoff delay calc
  const calculateNextDelay = (topic) => {
    const current = backoffDelays.current[topic] || 3000;
    const next = Math.min(current * 1.5, 30000);
    backoffDelays.current[topic] = next;
    return next;
  };

  // Reset reconnection state
  const resetReconnection = (topic) => {
    if (reconnectTimers.current[topic]) {
      clearTimeout(reconnectTimers.current[topic]);
      reconnectTimers.current[topic] = null;
    }
    reconnectAttempts.current[topic] = 0;
    backoffDelays.current[topic] = 3000;
    isConnectingRef.current[topic] = false;
  };

  // Connect function
  const connectTopic = useCallback((topic) => {
    // 연결 중이거나 이미 연결된 경우 패스
    if (isConnectingRef.current[topic]) {
      console.log(`WS connect skipped (already connecting): ${topic}`);
      return;
    }

    const existing = connectionsRef.current[topic];
    if (existing) {
      if (existing.readyState === WebSocket.OPEN) {
        console.log(`WS connect skipped (already open): ${topic}`);
        return;
      }
      if (existing.readyState === WebSocket.CONNECTING) {
        console.log(`WS connect skipped (connecting): ${topic}`);
        return;
      }
    }

    const attempts = reconnectAttempts.current[topic] || 0;
    if (attempts >= 5) {
      setErrors(e => ({ ...e, [topic]: '최대 재연결 시도 초과' }));
      return;
    }

    isConnectingRef.current[topic] = true;
    reconnectAttempts.current[topic] = attempts + 1;

    const url = `${WS_BASE_URL}/${topic}`;
    console.log(`WS connect: ${url}`);
    
    try {
      const ws = new WebSocket(url);
      connectionsRef.current[topic] = ws;

      // Timeout
      const timeoutId = setTimeout(() => {
        if (ws.readyState !== WebSocket.OPEN) {
          console.log(`WS connection timeout: ${topic}`);
          ws.close();
          isConnectingRef.current[topic] = false;
        }
      }, 5000);

      ws.onopen = () => {
        clearTimeout(timeoutId);
        console.log(`WS open: ${topic}`);
        setConnected(c => ({ ...c, [topic]: true }));
        setErrors(e => ({ ...e, [topic]: null }));
        resetReconnection(topic);
      };

      ws.onmessage = (evt) => {
        try {
          const msg = JSON.parse(evt.data);
          if (msg.type === 'update' && msg.topic === topic) {
            setData(d => ({ ...d, [topic]: msg.data }));
          }
          // console.log(`WS message: ${topic}`, msg);
        } catch (err) {
          console.error(`WS parse error (${topic}):`, err);
          setErrors(e => ({ ...e, [topic]: '메시지 파싱 오류' }));
        }
      };

      ws.onclose = (event) => {
        clearTimeout(timeoutId);
        console.log(`WS close: ${topic} code=${event.code}`);
        setConnected(c => ({ ...c, [topic]: false }));
        isConnectingRef.current[topic] = false;
        
        // 정상적으로 닫힌 경우 재연결 시도하지 않음
        if (event.code !== 1000 && event.code !== 1001) {
          // 이미 재연결 중이면 중복 타이머 방지
          if (reconnectTimers.current[topic]) {
            clearTimeout(reconnectTimers.current[topic]);
          }
          
          const delay = calculateNextDelay(topic);
          console.log(`Reconnecting ${topic} in ${delay}ms`);
          reconnectTimers.current[topic] = setTimeout(() => {
            connectTopic(topic);
          }, delay);
        }
      };

      ws.onerror = (err) => {
        console.error(`WS error: ${topic}`, err);
        setErrors(e => ({ ...e, [topic]: '연결 오류 발생' }));
        isConnectingRef.current[topic] = false;
        if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) {
          ws.close();
        }
      };
    } catch (err) {
      console.error(`WS creation error: ${topic}`, err);
      isConnectingRef.current[topic] = false;
    }
  }, []);

  // Manual refresh - close old connections and create new ones
  const refreshTopic = useCallback((topic) => {
    console.log(`Manual refresh requested for ${topic}`);
    
    // 기존 연결 정리
    const ws = connectionsRef.current[topic];
    if (ws) {
      if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) {
        console.log(`Closing existing connection for ${topic}`);
        ws.close(1000, 'manual refresh');
      }
      delete connectionsRef.current[topic];
    }

    // 재연결 타이머가 있으면 취소
    if (reconnectTimers.current[topic]) {
      clearTimeout(reconnectTimers.current[topic]);
      reconnectTimers.current[topic] = null;
    }

    // 연결 상태 초기화
    resetReconnection(topic);
    
    // 약간의 지연 후 새 연결 시도
    setTimeout(() => {
      connectTopic(topic);
    }, 500);
  }, [connectTopic]);

  // On mount: connect once
  useEffect(() => {
    // 모든 토픽에 대해 한 번만 연결
    stableTopics.forEach(topic => {
      setTimeout(() => {
        connectTopic(topic);
      }, Math.random() * 500); // 연결 요청 시간 분산
    });

    // Cleanup on unmount
    return () => {
      Object.entries(connectionsRef.current).forEach(([topic, ws]) => {
        if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
          console.log(`Closing WS on unmount: ${topic}`);
          ws.close(1000, 'unmount');
        }
      });
      Object.values(reconnectTimers.current).forEach(id => {
        if (id) clearTimeout(id);
      });
    };
  }, [connectTopic, stableTopics]);

  // Derive customer
  useEffect(() => {
    const t = data.tables;
    const o = data.orders;
    if (!t || !o) return;
    try {
      const tableId = parseInt(localStorage.getItem('kioskTableId') || '1');
      const assign = (t.assignments||[]).find(a =>
        a['GroupAssignment.table_id']===tableId && !a['GroupAssignment.released_at']
      );
      const cid = assign?.['GroupAssignment.customer_id'] || null;
      setCurrentCustomer(cid);
      if (cid) {
        const all = Array.isArray(o.orders)? o.orders.flat(): [];
        setCustomerOrders(all.filter(x=>
          x['Order.customer_id']===cid && x['Order.table_id']===tableId
        ));
      } else {
        setCustomerOrders([]);
      }
    } catch(e){ console.error('Customer error', e);}  
  }, [data.tables, data.orders]);

  const contextValue = { data, connected, errors, currentCustomer, customerOrders, refreshTopic };
  return <WSContext.Provider value={contextValue}>{children}</WSContext.Provider>;
}

export function useUnifiedWebSockets() {
  const ctx = useContext(WSContext);
  if (!ctx) throw new Error('Must use within provider');
  return ctx;
}
