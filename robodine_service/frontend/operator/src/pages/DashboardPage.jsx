import React, { useState, useEffect, useCallback } from 'react';
import Layout from '../components/Layout';
import RobotStatusPanel from '../components/dashboard/RobotStatusPanel';
import StoreMap from '../components/dashboard/StoreMap';
import EventTimeline from '../components/dashboard/EventTimeline';
import RecentOrders from '../components/dashboard/RecentOrders';
import SystemStatus from '../components/dashboard/SystemStatus';
import { useAuth } from '../contexts/AuthContext';

// 날짜 포맷팅 함수
const formatDateTime = (date) => {
  return date.toLocaleString('ko-KR', {
    year: 'numeric',
    month: 'long',
    day: 'numeric',
    weekday: 'long',
    hour: '2-digit',
    minute: '2-digit'
  });
};

// WebSocket 서버 URL - 백엔드 웹소켓 엔드포인트에 맞추어 수정
const WS_URL = 'ws://127.0.0.1:8000/ws/status';

const DashboardPage = () => {
  const [robots, setRobots] = useState([]);
  const [tables, setTables] = useState([]);
  const [events, setEvents] = useState([]);
  const [orders, setOrders] = useState([]);
  const [systems, setSystems] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  
  const [robotsError, setRobotsError] = useState(null);
  const [tablesError, setTablesError] = useState(null);
  const [eventsError, setEventsError] = useState(null);
  const [ordersError, setOrdersError] = useState(null);
  const [systemsError, setSystemsError] = useState(null);
  
  const [selectedEvent, setSelectedEvent] = useState(null);
  const [wsConnected, setWsConnected] = useState(false);
  
  const { apiCall } = useAuth();

  // WebSocket 연결 설정
  useEffect(() => {
    let ws = null;
    let reconnectAttempts = 0;
    const maxReconnectAttempts = 5;
    let reconnectTimeout = null;

    const connectWebSocket = () => {
      if (reconnectAttempts >= maxReconnectAttempts) {
        console.error('최대 재연결 시도 횟수를 초과했습니다.');
        // 웹소켓 상태 정보 추가
        setSystems(prevSystems => {
          const systemsArray = Array.isArray(prevSystems) ? [...prevSystems] : [];
          
          const websocketSystem = {
            id: 'websocket',
            name: '웹소켓 서버',
            type: 'WEBSOCKET',
            status: 'ERROR',
            description: '연결 실패: 최대 재연결 시도 횟수 초과',
            lastUpdated: new Date().toISOString()
          };
          
          return [...systemsArray.filter(sys => sys.id !== 'websocket'), websocketSystem];
        });
        return;
      }

      try {
        console.log('웹소켓 연결 시도 중...', WS_URL);
        ws = new WebSocket(WS_URL);

        ws.onopen = () => {
          console.log('WebSocket 연결 성공');
          setWsConnected(true);
          reconnectAttempts = 0;
          
          // 웹소켓 상태 정보 추가
          setSystems(prevSystems => {
            const systemsArray = Array.isArray(prevSystems) ? [...prevSystems] : [];
            
            const websocketSystem = {
              id: 'websocket',
              name: '웹소켓 서버',
              type: 'WEBSOCKET',
              status: 'ONLINE',
              description: '연결 성공 (데이터 수신 기능은 추후 구현 예정)',
              lastUpdated: new Date().toISOString()
            };
            
            return [...systemsArray.filter(sys => sys.id !== 'websocket'), websocketSystem];
          });
        };

        ws.onclose = () => {
          console.log('WebSocket 연결 종료');
          setWsConnected(false);
          
          // 웹소켓 상태 정보 업데이트
          setSystems(prevSystems => {
            const systemsArray = Array.isArray(prevSystems) ? [...prevSystems] : [];
            
            const websocketSystem = {
              id: 'websocket',
              name: '웹소켓 서버',
              type: 'WEBSOCKET',
              status: 'OFFLINE',
              description: '연결 종료됨',
              lastUpdated: new Date().toISOString()
            };
            
            return [...systemsArray.filter(sys => sys.id !== 'websocket'), websocketSystem];
          });
          
          // 재연결 시도
          reconnectAttempts++;
          const reconnectDelay = Math.min(1000 * Math.pow(2, reconnectAttempts), 30000);
          console.log(`${reconnectDelay / 1000}초 후 재연결 시도 (${reconnectAttempts}/${maxReconnectAttempts})...`);
          
          reconnectTimeout = setTimeout(connectWebSocket, reconnectDelay);
        };

        ws.onerror = (error) => {
          console.error('WebSocket 오류:', error);
        };

        ws.onmessage = (event) => {
          console.log('WebSocket 메시지 수신:', event.data);
          // 실제 데이터 처리는 추후 구현 예정
          // 현재는 메시지 수신만 로깅
        };
      } catch (error) {
        console.error('WebSocket 연결 오류:', error);
        reconnectAttempts++;
        
        // 웹소켓 상태 정보 업데이트
        setSystems(prevSystems => {
          const systemsArray = Array.isArray(prevSystems) ? [...prevSystems] : [];
          
          const websocketSystem = {
            id: 'websocket',
            name: '웹소켓 서버',
            type: 'WEBSOCKET',
            status: 'ERROR',
            description: `연결 실패: ${error.message}`,
            lastUpdated: new Date().toISOString()
          };
          
          return [...systemsArray.filter(sys => sys.id !== 'websocket'), websocketSystem];
        });
        
        // 재연결 시도
        const reconnectDelay = Math.min(1000 * Math.pow(2, reconnectAttempts), 30000);
        reconnectTimeout = setTimeout(connectWebSocket, reconnectDelay);
      }
    };

    connectWebSocket();

    // 정리 함수
    return () => {
      if (ws) {
        ws.close();
      }
      if (reconnectTimeout) {
        clearTimeout(reconnectTimeout);
      }
    };
  }, []);

  // API 데이터 가져오기
  useEffect(() => {
    const fetchDashboardData = async () => {
      setIsLoading(true);
      
      // Reset all error states
      setRobotsError(null);
      setTablesError(null);
      setEventsError(null);
      setOrdersError(null);
      setSystemsError(null);

      try {
        // Fetch robots data
        try {
          const robotsData = await apiCall('/api/robots');
          setRobots(robotsData);
        } catch (err) {
          console.error('Failed to fetch robots:', err);
          setRobotsError('로봇 정보를 불러올 수 없습니다');
        }

        // Fetch tables data
        try {
          const tablesData = await apiCall('/api/tables');
          setTables(tablesData);
        } catch (err) {
          console.error('Failed to fetch tables:', err);
          setTablesError('테이블 정보를 불러올 수 없습니다');
        }

        // Fetch events data
        try {
          const eventsData = await apiCall('/api/events');
          setEvents(eventsData);
        } catch (err) {
          console.error('Failed to fetch events:', err);
          setEventsError('이벤트 정보를 불러올 수 없습니다');
        }

        // Fetch orders data
        try {
          const ordersData = await apiCall('/api/orders');
          setOrders(ordersData);
        } catch (err) {
          console.error('Failed to fetch orders:', err);
          setOrdersError('주문 정보를 불러올 수 없습니다');
        }

        // 시스템 상태는 여기서 설정하지 않고 웹소켓 연결 상태만 유지
        setSystems(prevSystems => {
          // 기존 웹소켓 상태만 유지하고 나머지는 기본 시스템 정보로 설정
          const websocketSystem = Array.isArray(prevSystems) 
            ? prevSystems.find(s => s.id === 'websocket') 
            : null;
            
          const defaultSystems = [
            {
              id: 'server',
              name: '메인 서버',
              status: 'WARNING',
              type: 'SERVER',
              description: '시스템 상태 정보 가져오기 기능이 없습니다.',
              lastUpdated: new Date().toISOString()
            },
            {
              id: 'db',
              name: '데이터베이스',
              status: 'WARNING',
              type: 'DATABASE',
              description: '데이터베이스 상태 정보 가져오기 기능이 없습니다.',
              lastUpdated: new Date().toISOString()
            }
          ];
          
          return websocketSystem ? 
            [...defaultSystems, websocketSystem] : 
            defaultSystems;
        });
      } catch (err) {
        console.error('Error during dashboard data fetch:', err);
      } finally {
        setIsLoading(false);
      }
    };

    fetchDashboardData();

    // Set up polling for real-time updates every 30 seconds
    const intervalId = setInterval(fetchDashboardData, 30000);
    
    return () => clearInterval(intervalId);
  }, [apiCall]);

  const handleEventSelect = (event) => {
    setSelectedEvent(event);
  };

  const handleViewOrder = async (orderId) => {
    try {
      const orderDetails = await apiCall(`/api/orders/${orderId}`);
      console.log('Order details:', orderDetails);
    } catch (error) {
      console.error(`Error fetching order ${orderId}:`, error);
    }
  };

  return (
    <Layout>
      <div className="container mx-auto p-4">
        <div className="mb-6">
          <h1 className="text-2xl font-bold text-gray-800">대시보드</h1>
          <p className="text-gray-500 text-sm">{formatDateTime(new Date())}</p>
        </div>
        
        {isLoading ? (
          <div className="flex items-center justify-center h-64">
            <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
          </div>
        ) : (
          <div className="grid grid-cols-1 lg:grid-cols-4 gap-4">
            <div className="lg:col-span-2">
              <RobotStatusPanel 
                robots={robots} 
                isLoading={isLoading} 
                error={robotsError} 
              />
            </div>
            <div className="lg:col-span-2">
              <RecentOrders 
                orders={orders} 
                isLoading={isLoading} 
                error={ordersError} 
              />
            </div>
            <div className="lg:col-span-2">
              <EventTimeline 
                events={events} 
                isLoading={isLoading} 
                error={eventsError} 
                onSelectEvent={handleEventSelect} 
              />
            </div>
            <div className="lg:col-span-2">
              <SystemStatus 
                systems={systems} 
                isLoading={isLoading} 
                error={systemsError} 
              />
            </div>
            <div className="lg:col-span-4">
              <StoreMap 
                robots={robots} 
                tables={tables} 
                isLoading={isLoading} 
                robotsError={robotsError}
                tablesError={tablesError}
                selectedEvent={selectedEvent}
                wsConnected={wsConnected}
              />
            </div>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default DashboardPage; 