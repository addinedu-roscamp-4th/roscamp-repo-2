// robodine_service/frontend/operator/src/pages/DashboardPage.jsx

import React, { useState, useEffect, useCallback, useRef, useMemo } from 'react';
import Layout from '../components/Layout';
import RobotStatusPanel from '../components/dashboard/RobotStatusPanel';
import StoreMap from '../components/dashboard/StoreMap';
import EventTimeline from '../components/dashboard/EventTimeline';
import RecentOrders from '../components/dashboard/RecentOrders';
import { useAuth } from '../contexts/AuthContext';
import { QueryClient, QueryClientProvider } from 'react-query';
import { RefreshCw } from 'lucide-react';

// React Query 클라이언트 생성
const queryClient = new QueryClient();

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

// 웹소켓 서버 URL 설정
const WS_BASE_URL = 'ws://127.0.0.1:8000/ws';

// 매장 맵 테이블 위치 정의 (예시)
const TABLE_POSITIONS = {
  // 테이블 번호에 따른 x, y 좌표 (0-100 범위의 상대적 위치)
  1: { x: 34, y: 59 },
  2: { x: 48, y: 59 },
  3: { x: 34, y: 67 },
  4: { x: 48, y: 67 },
  // 5: { x: 40, y: 20 },
  // 6: { x: 40, y: 40 },
  // 7: { x: 40, y: 60 },
  // 8: { x: 40, y: 80 },
  // 9: { x: 60, y: 20 },
  // 10: { x: 60, y: 40 },
  // 11: { x: 60, y: 60 },
  // 12: { x: 60, y: 80 },
  // 13: { x: 80, y: 20 },
  // 14: { x: 80, y: 40 },
  // 15: { x: 80, y: 60 },
  // 16: { x: 80, y: 80 },
};

// 웹소켓 연결 상태 관리를 위한 커스텀 훅
const useWebSocket = (topic, onMessageReceived) => {
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  const wsRef = useRef(null);
  const reconnectTimeoutRef = useRef(null);
  const reconnectAttemptsRef = useRef(0);
  const heartbeatRef = useRef(null);
  const MAX_RECONNECT_ATTEMPTS = 5;
  const RECONNECT_INTERVAL = 3000; // 기본 재연결 간격 (ms)
  
  // 함수들을 useRef로 저장하여 순환 참조 문제 해결
  const connectRef = useRef(null);
  const scheduleReconnectRef = useRef(null);
  const startHeartbeatRef = useRef(null);
  
  // 하트비트 함수 정의
  startHeartbeatRef.current = () => {
    // 기존 하트비트 제거
    if (heartbeatRef.current) {
      clearInterval(heartbeatRef.current);
    }
    
    heartbeatRef.current = setInterval(() => {
      if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
        wsRef.current.send(JSON.stringify({
          type: 'ping',
          topic: topic,
          data: { timestamp: new Date().getTime() }
        }));
      } else {
        // 연결이 끊어진 경우 하트비트 중지
        clearInterval(heartbeatRef.current);
        heartbeatRef.current = null;
      }
    }, 30000); // 30초마다 핑 전송
  };
  
  // 재연결 스케줄링 함수 정의
  scheduleReconnectRef.current = () => {
    // 이미 재연결이 예약된 경우 중복 방지
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
    
    if (reconnectAttemptsRef.current >= MAX_RECONNECT_ATTEMPTS) {
      console.log(`${topic} 최대 재연결 시도 횟수 초과`);
      setError(`최대 재연결 시도 횟수(${MAX_RECONNECT_ATTEMPTS})를 초과했습니다.`);
      return;
    }
    
    // 지수 백오프 (2^attempt의 지연, 최대 30초)
    const delay = Math.min(RECONNECT_INTERVAL * Math.pow(2, reconnectAttemptsRef.current), 30000);
    console.log(`${topic} ${delay}ms 후 재연결 시도 (${reconnectAttemptsRef.current + 1}/${MAX_RECONNECT_ATTEMPTS})`);
    
    reconnectTimeoutRef.current = setTimeout(() => {
      reconnectAttemptsRef.current += 1;
      connectRef.current();
    }, delay);
  };
  
  // 연결 함수 정의
  connectRef.current = () => {
    try {
      // 기존 연결 및 타이머 정리
      if (wsRef.current) {
        wsRef.current.close();
      }
      
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
      
      // 새 웹소켓 연결 생성
      wsRef.current = new WebSocket(`${WS_BASE_URL}/${topic}`);
      
      // 연결 이벤트 핸들러
      wsRef.current.onopen = () => {
        console.log(`${topic} 웹소켓 연결됨`);
        setConnected(true);
        setError(null);
        reconnectAttemptsRef.current = 0;
        
        // 연결 유지를 위한 핑-퐁
        startHeartbeatRef.current();
      };
      
      // 메시지 수신 핸들러
      wsRef.current.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          
          // 핑-퐁 처리
          if (data.type === 'pong') {
            return;
          }
          
          // 서버 종료 메시지 처리
          if (data.type === 'shutdown') {
            console.log(`서버 종료 메시지 수신: ${data.data?.reason || '알 수 없는 이유'}`);
            setError(`서버가 종료됩니다: ${data.data?.reason || '알 수 없는 이유'}`);
            return;
          }
          
          // 업데이트 메시지 처리 (콜백으로 상위 컴포넌트에 전달)
          if (data.type === 'update' && data.topic === topic) {
            onMessageReceived(data.data);
          }
        } catch (error) {
          console.error(`${topic} 웹소켓 메시지 처리 오류:`, error);
        }
      };
      
      // 연결 종료 핸들러
      wsRef.current.onclose = (event) => {
        console.log(`${topic} 웹소켓 연결 종료:`, event.code, event.reason);
        setConnected(false);
        
        // 하트비트 정리
        if (heartbeatRef.current) {
          clearInterval(heartbeatRef.current);
          heartbeatRef.current = null;
        }
        
        // 정상 종료가 아닌 경우에만 재연결 시도
        if (!event.wasClean) {
          scheduleReconnectRef.current();
        }
      };
      
      // 오류 핸들러
      wsRef.current.onerror = (error) => {
        console.error(`${topic} 웹소켓 오류:`, error);
        setError(`연결 오류: ${error.message || '알 수 없는 오류'}`);
        
        // 오류 발생 시 웹소켓 종료 (onclose 이벤트가 발생하여 재연결 처리)
        if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
          wsRef.current.close();
        }
      };
    } catch (error) {
      console.error(`${topic} 웹소켓 연결 생성 오류:`, error);
      setError(`연결 생성 오류: ${error.message || '알 수 없는 오류'}`);
      scheduleReconnectRef.current();
    }
  };
  
  
  // 래퍼 함수를 메모이제이션
  const connect = useCallback(() => {
    connectRef.current();
  }, []);
  
  // 컴포넌트 마운트 시 최초 연결
  useEffect(() => {
    connect();
    
    // 컴포넌트 언마운트 시 정리
    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
      if (heartbeatRef.current) {
        clearInterval(heartbeatRef.current);
      }
    };
  }, [connect]);
  
  // 연결 상태, 오류, 수동 재연결 함수 반환
  return { connected, error, reconnect: connect };
};

const DashboardPage = () => {
  // 데이터 상태
  const [robotsData, setRobotsData] = useState([]);
  const [robotDetailsData, setRobotDetailsData] = useState({
    albabots: [],
    cookbots: []
  });
  const [tablesData, setTablesData] = useState([]);
  const [eventsData, setEventsData] = useState([]);

  const [ordersData, setOrdersData] = useState({
    orders: [], kioskterminals: [], orderItems: [], menuitems: []
  });  

  const [poseData, setPoseData] = useState([]);
  const [poseLatencies, setPoseLatencies] = useState({});

  
  const [lastUpdateTime, setLastUpdateTime] = useState(new Date());
  const [isLoading, setIsLoading] = useState(true);

  const { apiCall: _apiCall } = useAuth(); // eslint-disable-line no-unused-vars

  // 디버깅을 위한 상태 로깅 함수
  const logDataState = useCallback(() => {
    // console.log("로봇 데이터:", robotsData);
    // console.log("로봇 상세 데이터:", robotDetailsData);
    // console.log("테이블 데이터:", tablesData);
    console.log("주문 데이터:", ordersData);
    // console.log("이벤트 데이터:", eventsData);
    // console.log("위치 데이터:", poseData);
  }, [robotsData, robotDetailsData, tablesData, ordersData, eventsData, poseData]);

  // 로봇 상태 메시지 핸들러
  const handleRobotsMessage = useCallback((data) => {
    console.log("로봇 웹소켓 데이터 수신:", data);
    
    if (!Array.isArray(data)) {
      console.warn('로봇 데이터가 배열 형태가 아닙니다:', data);
      return;
    }
    
    try {
      const formattedData = data.map(robot => {
        const baseInfo = {
          id: robot['Robot.id'] || robot.id,
          robot_id: robot['Robot.robot_id'] || robot.robot_id,
          type: robot['Robot.type'] || robot.type,
          mac_address: robot['Robot.mac_address'] || robot.mac_address,
          ip_address: robot['Robot.ip_address'] || robot.ip_address,
          timestamp: robot['Robot.timestamp'] || robot.timestamp
        };
        
        if (baseInfo.type === 'ALBABOT') {
          baseInfo.status = robot['Albabot.status'] || 'Not_Connected';
          baseInfo.battery = robot['Albabot.battery_level'] !== undefined ? 
            Math.round(robot['Albabot.battery_level'] * 100) : 0;
        } else if (baseInfo.type === 'COOKBOT') {
          baseInfo.status = robot['Cookbot.status'] || 'Not_Connected';
          baseInfo.battery = 100; // 쿡봇은 유선 전원 가정
        }
        // 로그를 주석 처리하여 중복 출력 방지
        // console.log("변환 전 로봇 데이터:", baseInfo);
        return baseInfo;
      });

      // 기존 로봇 데이터가 있으면 해당 데이터의 추가 정보(배터리, 상태, 위치 등)를 유지
      setRobotsData(prevRobots => {
        // 기존 데이터 기반으로 새 데이터 병합
        const mergedData = formattedData.map(newRobot => {
          const existingRobot = prevRobots.find(r => r.id === newRobot.id);
          if (existingRobot) {
            // 기존 로봇 데이터의 중요 정보 유지 (위치, 상태, 배터리)
            return {
              ...newRobot,
              position: existingRobot.position || newRobot.position,
              status: existingRobot.status !== 'Not_Connected' ? existingRobot.status : newRobot.status,
              battery: existingRobot.battery > 0 ? existingRobot.battery : newRobot.battery
            };
          }
          return newRobot;
        });
        
        return mergedData;
      });
      
      setLastUpdateTime(new Date());
      setIsLoading(false);
    } catch (error) {
      console.error("로봇 데이터 처리 오류:", error);
    }
  }, []);

  // 로봇 상세 메시지 핸들러
  const handleRobotDetailsMessage = useCallback((data) => {
    console.log("로봇 상세 데이터 수신:", data);

    if (data && typeof data === 'object') {
      // 로봇 상세 데이터 업데이트
      setRobotDetailsData(prevData => ({
        ...prevData,
        albabots: Array.isArray(data.albabots) ? data.albabots : prevData.albabots,
        cookbots: Array.isArray(data.cookbots) ? data.cookbots : prevData.cookbots
      }));

      if (robotsData.length > 0 && (Array.isArray(data.albabots) || Array.isArray(data.cookbots))) {
        setRobotsData(prevRobots => {
          const updatedRobots = prevRobots.map(robot => {
            let updatedRobot = { ...robot };

            if (robot.type === 'ALBABOT' && Array.isArray(data.albabots)) {
              const matchingAlbabot = data.albabots.find(albabot => albabot["Albabot.robot_id"] === robot.robot_id);
              if (matchingAlbabot) {
                // console.log("매칭된 알바봇 데이터:", matchingAlbabot);
                updatedRobot.status = matchingAlbabot["Albabot.status"] || robot.status;
                
                // 배터리 레벨이 0-1 사이의 값이면 퍼센트로 변환 (100 곱하기)
                const batteryLevel = matchingAlbabot["Albabot.battery_level"];
                if (batteryLevel !== undefined) {
                  updatedRobot.battery = batteryLevel <= 1 ? 
                    Math.round(batteryLevel * 100) : Math.round(batteryLevel);
                }
              }
            }

            if (robot.type === 'COOKBOT' && Array.isArray(data.cookbots)) {
              const matchingCookbot = data.cookbots.find(cookbot => cookbot["Cookbot.robot_id"] === robot.robot_id);
              if (matchingCookbot) {
                // console.log("매칭된 쿡봇 데이터:", matchingCookbot);
                updatedRobot.status = matchingCookbot["Cookbot.status"] || robot.status;
              }
            }

            return updatedRobot;
          });

          // 상태 업데이트 후 로깅
          // console.log("로봇 데이터 상태 업데이트:", updatedRobots);
          return updatedRobots;
        });
      }

      setLastUpdateTime(new Date());
    }
  }, [robotsData]);

  useEffect(() => {
    // 모든 데이터가 처리된 후에만 로깅 (조건 완화)
    if (robotsData.length > 0) {
      // console.log("로봇 데이터 상태 업데이트2:", robotsData);
      
      // 완전한 데이터가 있는 경우에만 추가 로그 표시
      if (robotsData.some(robot => robot.position && robot.battery > 0)) {
        // console.log("모든 데이터가 반영된 알바봇 상태:", robotsData);
      }
    }
  }, [robotsData]);

  // 위치 데이터 수신 핸들러
  const handlePoseMessage = useCallback((data) => {
    // console.log("위치 데이터 수신:", data);
    
    if (Array.isArray(data)) {
      // 로봇 ID로 그룹화
      const posesByEntity = data.reduce((acc, pose) => {
        // Pose6D 테이블에서 entity_id와 entity_type 필드 확인
        const entityId = pose['Pose6D.entity_id'] || pose.entity_id;
        const entityType = pose['Pose6D.entity_type'] || pose.entity_type;
        
        // 세계 좌표계 위치만 사용 (entity_type이 'WORLD')
        if (entityType === 'WORLD') {
          const ts = new Date(pose['Pose6D.timestamp'] || pose.timestamp).getTime();
          const now = Date.now();
          if (!acc[entityId]) {
            acc[entityId] = [];
          }
          acc[entityId].push({
            x: pose['Pose6D.x'] || pose.x || 0,
            y: pose['Pose6D.y'] || pose.y || 0,
            z: pose['Pose6D.z'] || pose.z || 0,
            timestamp: ts,
          });
          setPoseLatencies(lat => ({ ...lat, [entityId]: now - ts }));
        }
        return acc;
      }, {});
      
      setPoseData(posesByEntity);
      
      // 위치 정보를 로봇 데이터에 통합
      if (robotsData.length > 0) {
        setRobotsData(prevRobots => {
          return prevRobots.map(robot => {
            // robot.id 또는 entity_id 값으로 위치 데이터 매핑
            const robotPoses = posesByEntity[robot.id];
            if (robotPoses && robotPoses.length > 0) {
              // 가장 최근 위치 사용
              const latestPose = robotPoses.sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp))[0];
              return {
                ...robot,
                position: {
                  x: latestPose.x,
                  y: latestPose.y,
                  z: latestPose.z
                }
              };
            }
            return robot;
          });
        });
      }
      
      setLastUpdateTime(new Date());
    }
  }, [robotsData]);

  // 테이블 데이터 수신 핸들러
  const handleTablesMessage = useCallback((data) => {
    // console.log("테이블 데이터 수신:", data);
    
    if (Array.isArray(data)) {
      // 테이블 데이터 포맷팅
      const formattedTables = data.map(table => {
        return {
          id: table['Table.id'] || table.id,
          tableNumber: table['Table.table_number'] || table.table_number,
          maxCustomer: table['Table.max_customer'] || table.max_customer,
          status: table['Table.status'] || table.status,
          updatedAt: table['Table.updated_at'] || table.updated_at,
          // 매장 맵에 표시하기 위한 위치 정보 추가
          position: TABLE_POSITIONS[table['Table.table_number'] || table.table_number] || { x: 0, y: 0 }
        };
      });
      
      setTablesData(formattedTables);
      setLastUpdateTime(new Date());
    }
  }, []);

  // 주문 데이터 수신 핸들러
  const handleOrdersMessage = useCallback((data) => {
    // console.log("주문 데이터 수신:", data);
    // console.log("주문 데이터:", data.orders);
    // console.log("주문 항목 데이터:", data.orderitems);
    // console.log("키오스크 데이터:", data.kioskterminals);
    // console.log("메뉴 데이터:", data.menuitems)
    const rawOrders = Array.isArray(data.orders)
    ? data.orders.flat()
    : [];
    setOrdersData({
    orders: rawOrders,
    kioskterminals: data.kioskterminals,
    orderItems: data.orderitems,
    menuitems: data.menuitems
  });
}, []);
  

  // 이벤트 데이터 수신 핸들러
  const handleEventsMessage = useCallback((data) => {
    // console.log("이벤트 데이터 수신:", data);
    
    if (Array.isArray(data)) {
      // 이벤트 데이터 포맷팅
      const formattedEvents = data.map(event => {
        return {
          id: event['Event.id'] || event.id,
          type: event['Event.type'] || event.type,
          related_entity_type: event['Event.related_entity_type'] || event.related_entity_type,
          related_entity_id: event['Event.related_entity_id'] || event.related_entity_id,
          description: event['Event.description'] || event.description,
          timestamp: event['Event.timestamp'] || event.timestamp,
          // UI에 표시할 메시지 및 위치 정보 추가
          message: event['Event.description'] || event.description || '이벤트 발생',
          location: event['Event.related_entity_type'] === 'ALBABOT' || event['Event.related_entity_type'] === 'COOKBOT' 
            ? '로봇 위치' 
            : '매장 내',
          robotId: event['Event.related_entity_type'] === 'ALBABOT' || event['Event.related_entity_type'] === 'COOKBOT'
            ? event['Event.related_entity_id']
            : null
        };
      });
      
      setEventsData(formattedEvents);
      setLastUpdateTime(new Date());
    }
  }, []);

  // 웹소켓 연결 및 사용
  const robotsWS = useWebSocket('robots', handleRobotsMessage);
  const tablesWS = useWebSocket('tables', handleTablesMessage);
  const eventsWS = useWebSocket('events', handleEventsMessage);
  
  // status 토픽에서 로봇 상세 정보와 위치 정보 수신
  const statusWS = useWebSocket('status', (data) => {
    // console.log("상태 데이터 수신:", data);
    
    // 데이터 타입에 따라 적절한 핸들러 호출
    if (data && typeof data === 'object') {
      // 로봇 상세 정보 (알바봇, 쿡봇) 처리
      if (data.albabots || data.cookbots) {
        handleRobotDetailsMessage(data);
      }
      
      // 위치 정보 처리
      if (data.poses && Array.isArray(data.poses)) {
        handlePoseMessage(data.poses);
      } else if (Array.isArray(data)) {
        handlePoseMessage(data);
      }
      
      if (data.robots && data.albabots && data.cookbots && data.poses) {
        // console.log("통합 데이터 수신:", data);
        
        // 로봇 상세 데이터 처리
        handleRobotDetailsMessage({
          albabots: data.albabots,
          cookbots: data.cookbots
        });
        
        // 위치 데이터 처리
        if (Array.isArray(data.poses)) {
          handlePoseMessage(data.poses);
        }
        
        // 선택적으로 robots 데이터도 처리 가능
        if (Array.isArray(data.robots) && data.robots.length > 0) {
          handleRobotsMessage(data.robots);
        }
      }
    }
  });

  const ordersWS = useWebSocket('orders', (data) => {
    // console.log("주문 데이터 수신:", data);
    
    if (data && typeof data === 'object') {
      handleOrdersMessage({
        orders: data.orders || [],
        orderitems: data.orderitems || [],
        kioskterminals: data.kioskterminals || [],
        menuitems: data.menuitems || []
      });
    }
  });

  // 로딩 상태 관리 (2초 후 로딩 완료로 간주)
  useEffect(() => {
    const timer = setTimeout(() => {
      setIsLoading(false);
    }, 2000);
    
    return () => clearTimeout(timer);
  }, []);

  // 5초마다 데이터 상태 로깅 (디버깅용)
  useEffect(() => {
    const logTimer = setInterval(() => {
      logDataState();
    }, 5000);
    
    return () => clearInterval(logTimer);
  }, [logDataState]);

  // UI에 표시할 로봇 데이터 처리
  const processedRobots = useMemo(() => {
    return robotsData.map(robot => {
      // 위치 정보 가져오기
      const position = robot.position || { x: 0, y: 0 };
      
      // 로봇 이름은 robot.type에서 'EntityTpye'을 뺀 후 id를 붙여서 생성
      const name = robot.type.replace(/EntityType./g, '').replace(/_/g, ' ');
      
      return {
        id: robot.id,
        name: `${name} #${robot.id}`,
        type: robot.type,
        status: robot.status || 'Not_Connected',
        battery: robot.battery || 0,
        ipAddress: robot.ip_address,
        macAddress: robot.mac_address,
        lastActive: robot.timestamp,
        position: position
      };
    });
  }, [robotsData]);

  // UI에 표시할 테이블 데이터 처리
  const processedTables = useMemo(() => {
    return tablesData.map(table => {
      // 테이블 위치 정보 가져오기
      const position = table.position || TABLE_POSITIONS[table.tableNumber] || { x: 0, y: 0 };
      
      return {
        id: table.id,
        tableNumber: table.tableNumber,
        status: table.status,
        maxCustomer: table.maxCustomer,
        updatedAt: table.updatedAt,
        position: position
      };
    });
  }, [tablesData]);

  // UI에 표시할 주문 데이터 처리
  const processedOrders = useMemo(() => {
    console.log("가공전 데이터:", ordersData);
    return ordersData.orders.map(order => {
      // 원본 ISO 문자열
      const raw = order['Order.timestamp'];     // e.g. "2025-04-30T12:57:58.297108"
      // 소수점(.) 이후 부분을 모두 잘라내고, "YYYY-MM-DDTHH:mm:ss" 형태로
      const cleaned = raw ? raw.split('.')[0] : '';
      
      // 주문 항목과 메뉴 항목을 매칭
      const matchingOderItem = ordersData.orderItems.find(i => i['OrderItem.order_id'] === order['Order.id']);
      const mathchingMenuItem = ordersData.menuitems.find(i => i['MenuItem.id'] === matchingOderItem['OrderItem.menu_item_id']);
      const matchingKiosk = ordersData.kioskterminals.find(i => i['KioskTerminal.id'] === order['Order.kiosk_id']);

      // console.log("matchingOderItem:", matchingOderItem);
      // console.log("mathchingMenuItem:", mathchingMenuItem);
      // console.log("matchingKiosk:", matchingKiosk);
      // console.log("menuid:", matchingOderItem['OrderItem.menu_item_id']);
      // console.log("quantity:", matchingOderItem['OrderItem.quantity']);
      // console.log("menuname:", mathchingMenuItem['MenuItem.name']);
      // console.log("price:", mathchingMenuItem['MenuItem.price']);

      return {
        id:        order['Order.id'],
        status:    order['Order.status'],
        table:     order['Order.table_id'],
        timestamp: cleaned,                     // 이제 "2025-04-30T12:57:58"
        // items에 orderitem의 id, 수량과 menuitem의 이름, 가격을 포함
        items:
          matchingOderItem ? [{
            id: matchingOderItem['OrderItem.menu_item_id'],
            quantity: matchingOderItem['OrderItem.quantity'],
            name: mathchingMenuItem ? mathchingMenuItem['MenuItem.name'] : 'Unknown',
            price: mathchingMenuItem ? mathchingMenuItem['MenuItem.price'] : 0
          }] : [],
        // totalpric는 items의 가격 합계
        totalPrice: matchingOderItem ? matchingOderItem['OrderItem.quantity'] * (mathchingMenuItem ? mathchingMenuItem['MenuItem.price'] : 0) : 0,
        totalQuantity: matchingOderItem ? matchingOderItem['OrderItem.quantity'] : 0,
      };
    });
  }, [ordersData]);

  // 최근 주문 정렬 (시간순)
  const recentOrders = useMemo(() => {
    console.log("가공된 주문 데이터:", processedOrders);
    return [...processedOrders].sort((a, b) => 
      new Date(b.timestamp) - new Date(a.timestamp)
    ).slice(0, 10); // 최근 10개의 주문만 표시
 }, [processedOrders]);

  // UI에 표시할 이벤트 데이터 처리
  const processedEvents = useMemo(() => {
    return eventsData.map(event => {
      return {
        id: event.id,
        type: event.type,
        message: event.description || `${event.type} 이벤트`,
        timestamp: event.timestamp,
        location: event.location,
        robotId: event.robotId,
        related_entity_type: event.related_entity_type,
        related_entity_id: event.related_entity_id
      };
    });
  }, [eventsData]);

  // 최근 이벤트 정렬 (시간순)
  const recentEvents = useMemo(() => {
    return [...processedEvents].sort((a, b) => 
      new Date(b.timestamp) - new Date(a.timestamp)
    ).slice(0, 10); // 최근 10개만 표시
  }, [processedEvents]);

  // 데이터 수동 갱신 요청
  const handleRefreshData = useCallback(() => {
    setIsLoading(true);
    
    // 각 웹소켓 연결 재연결 시도
    robotsWS.reconnect();
    tablesWS.reconnect();
    eventsWS.reconnect();
    ordersWS.reconnect();
    statusWS.reconnect();
    
    // 2초 후 로딩 완료
    setTimeout(() => {
      setIsLoading(false);
    }, 2000);
  }, [robotsWS, tablesWS, eventsWS, ordersWS, statusWS]);

  return (
    <Layout>
    <div className="h-full w-full flex flex-col">
  {/* 헤더 영역: 고정 높이 */}
  <header className="flex-shrink-0 px-4 py-2 bg-white shadow-md flex items-center justify-between">
    <h1 className="text-2xl font-bold text-gray-800">대시보드</h1>
    <div className="flex items-center space-x-4">
      <p className="text-sm text-gray-500">
        마지막 업데이트: {formatDateTime(lastUpdateTime)}
      </p>
      <button
        onClick={handleRefreshData}
        className="p-2 bg-white border border-gray-300 rounded-md hover:bg-gray-50"
        disabled={isLoading}
      >
        <RefreshCw size={16} className={isLoading ? 'animate-spin' : ''} />
      </button>
    </div>
  </header>

  {/* 메인 컨텐츠: 패널들이 화면에 딱 맞게 배치 */}
  <main className="flex-1 grid grid-cols-12 grid-rows-[32rem_1fr] gap-4 p-4 h-full overflow-hidden">
  {/* 매장 맵 */}
    <section className="col-span-12 lg:col-span-8 h-full">
      <StoreMap
        tables={processedTables}
        robots={processedRobots.map(r => ({ ...r, latency: poseLatencies[r.id] }))}
        isLoading={isLoading}
        wsConnected={robotsWS.connected}
      />
    </section>
    {/* 로봇 상태 패널 */}
    <section className="col-span-12 lg:col-span-4 flex flex-col min-h-0">
        <RobotStatusPanel
        className="h-full"
        robots={processedRobots}
        error={robotsWS.error}
        isLoading={isLoading}
      />
    </section>
    {/* 이벤트 타임라인 */}
    <section className="col-span-12 lg:col-span-6 flex flex-col min-h-0">
    <EventTimeline
        className="flex-1 overflow-auto"
        events={recentEvents}
        error={eventsWS.error}
        isLoading={isLoading}
      />
    </section>
    {/* 최근 주문 리스트 */}
    <section className="col-span-12 lg:col-span-6 flex flex-col min-h-0">
    <div className="flex-1 overflow-y-auto">
    <RecentOrders orders={recentOrders} />
  </div>
</section>
  </main>
</div>
  </Layout>
  );
};

// 래퍼 컴포넌트: React Query Provider 적용
const DashboardPageWrapper = () => (
  <QueryClientProvider client={queryClient}>
      <DashboardPage />
  </QueryClientProvider>
);

export default DashboardPageWrapper;