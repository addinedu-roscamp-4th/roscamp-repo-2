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
  // 주문 항목별 남은 시간을 관리하는 상태
  const [orderItemsRemainingTime, setOrderItemsRemainingTime] = useState({});

  // 타이머 ref
  const timersRef = useRef({});

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
            
            // 주문 항목 상태가 변경되었을 때 타이머 처리
            if (topic === 'orders' && msg.data && msg.data.orderitems) {
              processOrderItemUpdates(msg.data.orderitems);
            }
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

  // 주문 항목 업데이트 처리 함수
  const processOrderItemUpdates = useCallback((orderItems) => {
    // 현재 타이머 중지
    Object.keys(timersRef.current).forEach(itemId => {
      clearInterval(timersRef.current[itemId]);
      delete timersRef.current[itemId];
    });

    if (!Array.isArray(orderItems)) return;

    // 주문 항목의 상태 확인 및 타이머 업데이트
    orderItems.forEach(item => {
      const itemId = item['OrderItem.id'] || item.id;
      const orderItemStatus = item['OrderItem.status'] || item.status;
      const menuItemId = item['OrderItem.menu_item_id'] || item.menu_item_id;
      
      // 메뉴 아이템 정보 찾기 (prepare_time 가져오기 위해)
      if (data.orders && data.orders.menuitems) {
        const menuItem = data.orders.menuitems.find(mi => 
          (mi['MenuItem.id'] || mi.id) === menuItemId
        );
        
        if (menuItem) {
          const prepareTime = menuItem['MenuItem.prepare_time'] || 5; // 기본값 5분
          
          // 상태 확인 및 타이머 시작
          if (orderItemStatus === 'PREPARING' || orderItemStatus === 'COOKING') {
            // 이미 시작된 타이머가 없으면 새 타이머 시작
            if (!orderItemsRemainingTime[itemId]) {
              // 초기 남은 시간 설정 (분 단위)
              setOrderItemsRemainingTime(prev => ({
                ...prev,
                [itemId]: prepareTime
              }));
              
              // 타이머 시작 (60초마다 1분씩 감소)
              timersRef.current[itemId] = setInterval(() => {
                setOrderItemsRemainingTime(prev => {
                  if (!prev[itemId] || prev[itemId] <= 0) {
                    clearInterval(timersRef.current[itemId]);
                    delete timersRef.current[itemId];
                    return prev;
                  }
                  
                  const newValue = {
                    ...prev,
                    [itemId]: Math.max(0, prev[itemId] - 1)
                  };
                  return newValue;
                });
              }, 60000); // 60초마다 1분씩 감소
            }
          } else if (orderItemStatus === 'SERVED' || orderItemStatus === 'COMPLETED') {
            // 서빙 완료된 경우 타이머 제거
            if (timersRef.current[itemId]) {
              clearInterval(timersRef.current[itemId]);
              delete timersRef.current[itemId];
            }
            
            // 남은 시간을 0으로 설정
            setOrderItemsRemainingTime(prev => ({
              ...prev,
              [itemId]: 0
            }));
          }
        }
      }
    });
  }, [data.orders]);

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
      // 모든 타이머 정리
      Object.values(timersRef.current).forEach(id => {
        if (id) clearInterval(id);
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

  const contextValue = { 
    data, 
    connected, 
    errors, 
    currentCustomer, 
    customerOrders, 
    refreshTopic,
    orderItemsRemainingTime // 남은 시간 상태 추가
  };
  
  return <WSContext.Provider value={contextValue}>{children}</WSContext.Provider>;
}

export function useUnifiedWebSockets() {
  const ctx = useContext(WSContext);
  if (!ctx) throw new Error('Must use within provider');
  return ctx;
}
