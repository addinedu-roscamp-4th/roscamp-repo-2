// src/contexts/WebSocketContext.jsx
import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';

const WS_BASE_URL = 'ws://127.0.0.1:8000/ws';
const TOPICS = ['robots', 'tables', 'events', 'orders', 'status', 'systemlogs'];

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
    robots: [], tables: [], events: [], orders: {}, status: {}, systemlogs: []
  });
  const [errors, setErrors] = useState({});
  const [connected, setConnected] = useState({});

  // Connect to a single topic (stable callback)
  const connectTopic = useCallback((topic) => {
    const existing = connections[topic];
    if (existing && (existing.readyState === WebSocket.OPEN || existing.readyState === WebSocket.CONNECTING)) {
      return;
    }
    console.log(`WebSocket: connecting to ${topic}`);
    const ws = new WebSocket(`${WS_BASE_URL}/${topic}`);
    ws.onopen = () => {
      console.log(`WebSocket: ${topic} connected`);
      setConnected(prev => ({ ...prev, [topic]: true }));
    };
    ws.onmessage = (evt) => {
      console.log(`WebSocket: ${topic} message`, evt.data);
      try {
        const msg = JSON.parse(evt.data);
        if (msg.type === 'update' && msg.topic === topic) {
          setData(prev => ({ ...prev, [topic]: msg.data }));
        }
      } catch (err) {
        console.error(`WebSocket: ${topic} parse error`, err);
        setErrors(prev => ({ ...prev, [topic]: err }));
      }
    };
    const RECONNECT_DELAY = 3000;
    ws.onclose = () => {
      console.warn(`WebSocket: ${topic} closed, reconnecting in ${RECONNECT_DELAY}ms`);
      setConnected(prev => ({ ...prev, [topic]: false }));
      setTimeout(() => connectTopic(topic), RECONNECT_DELAY);
    };
    ws.onerror = (err) => {
      console.error(`WebSocket: ${topic} error`, err);
      setErrors(prev => ({ ...prev, [topic]: err }));
    };
    setConnections(prev => ({ ...prev, [topic]: ws }));
  }, []); // 빈 deps → stable reference

  // Refresh connection for a topic
  const refreshTopic = useCallback((topic) => {
    connectTopic(topic);
  }, [connectTopic]);

  // Setup connections once on mount
  useEffect(() => {
    TOPICS.forEach(connectTopic);
    return () => {
      Object.values(connections).forEach(ws => {
        try { ws.close(); } catch {}
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
