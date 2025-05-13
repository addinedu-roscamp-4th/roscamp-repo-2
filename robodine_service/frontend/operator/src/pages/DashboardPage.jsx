// src/pages/DashboardPage.jsx
import React, { useState, useEffect, useMemo, useCallback } from 'react';
import Layout from '../components/Layout';
import RobotStatusPanel from '../components/dashboard/RobotStatusPanel';
import StoreMap from '../components/dashboard/StoreMap';
import EventTimeline from '../components/dashboard/EventTimeline';
import RecentOrders from '../components/dashboard/RecentOrders';
import { RefreshCw } from 'lucide-react';
import { useWebSockets } from '../contexts/WebSocketContext';

// 웹소켓 토픽 리스트 (Context와 동일하게 유지)
const TOPICS = ['robots', 'tables', 'events', 'orders', 'status', 'systemlogs'];

const formatDateTime = date =>
  date.toLocaleString('ko-KR', {
    year: 'numeric', month: 'long', day: 'numeric',
    weekday: 'long', hour: '2-digit', minute: '2-digit'
  });

// 테이블 기본 위치 지정 - CustomerPage와 같은 좌표계 사용
const TABLE_POSITIONS = {
  1: { x: 34, y: 59 }, 2: { x: 48, y: 59 },
  3: { x: 34, y: 67 }, 4: { x: 48, y: 67 },
};

export default function DashboardPage() {
  const { data, errors, connected, refreshTopic } = useWebSockets();
  const { robots, tables, events, orders, status, systemlogs } = data;

  const [lastUpdateTime, setLastUpdateTime] = useState(new Date());
  const [isLoading, setIsLoading] = useState(true);
  const [poseLatencies, setPoseLatencies] = useState({});
  const [selectedOrderId, setSelectedOrderId] = useState(null);
  const [isOrderDetailModalOpen, setIsOrderDetailModalOpen] = useState(false);

  // 로봇 데이터 처리
  const processedRobots = useMemo(() => {
    // status.robots 배열이 있으면 이걸 쓰고, 없으면 fallback
    const base = Array.isArray(status.robots)
      ? status.robots
      : Array.isArray(robots)
        ? robots
        : [];
  
    return base.map(r => {
      // 1) 기본 필드
      const robot = {
        id: r['Robot.id'],
        robot_id: r['Robot.robot_id'],
        type: r['Robot.type'],
        ipAddress: r['Robot.ip_address'],
        macAddress: r['Robot.mac_address'],
        lastActive: r['Robot.timestamp'],
      };
  
      // 2) albabot/cookbot 상태
      const alb = (status.albabots || []).find(a => a['Albabot.robot_id'] === r['Robot.robot_id']);
      if (alb) {
        robot.status = alb['Albabot.status'];
        const lvl = alb['Albabot.battery_level'];
        robot.battery = lvl <= 1 ? Math.round(lvl * 100) : Math.round(lvl);
      } else {
        const cook = (status.cookbots || []).find(c => c['Cookbot.robot_id'] === r['Robot.robot_id']);
        if (cook) {
          robot.status = cook['Cookbot.status'];
          robot.battery = 100;
        } else {
          robot.status = 'Not_Connected';
          robot.battery = 0;
        }
      }
  
      // 3) 위치 (poses)
      const pose = (status.poses || []).find(p => p['Pose6D.entity_id'] === r['Robot.id']);
      if (pose) {
        robot.position = {
          x: pose['Pose6D.x'],
          y: pose['Pose6D.y'],
          z: pose['Pose6D.z']
        };
        // optional: latency 계산
        const ts = new Date(pose['Pose6D.timestamp']).getTime();
        setPoseLatencies(lat => ({ ...lat, [robot.id]: Date.now() - ts }));
      }
  
      robot.name = `${robot.type.replace(/_/g,' ')} #${robot.id}`;
      return robot;
    });
  }, [status, robots]);

  // 테이블 배정 정보 처리 (CustomerPage와 동일한 방식)
  const processedAssignmentsData = useMemo(() => {
    // 배정 정보 가져오기 (released_at이 null인 활성 배정만)
    const tableAssignments = data?.tables?.assignments && Array.isArray(data.tables.assignments)
      ? data.tables.assignments
          .filter(a => a['GroupAssignment.released_at'] === null)
          .map((a) => ({
            id: a['GroupAssignment.id'],
            table_id: a['GroupAssignment.table_id'],
            customer_id: a['GroupAssignment.customer_id'],
            timestamp: a['GroupAssignment.timestamp'],
          }))
      : [];
    
    const customerAssignments = data?.customers?.assignments && Array.isArray(data.customers.assignments)
      ? data.customers.assignments
          .filter(a => a['GroupAssignment.released_at'] === null)
          .map((a) => ({
            id: a['GroupAssignment.id'],
            table_id: a['GroupAssignment.table_id'],
            customer_id: a['GroupAssignment.customer_id'],
            timestamp: a['GroupAssignment.timestamp'],
          }))
      : [];
    
    // 중복 제거
    const merged = [...tableAssignments, ...customerAssignments];
    const deduplicated = merged.filter((assignment, index, self) =>
      index === self.findIndex((a) => a && a.id === assignment.id)
    );
    
    return deduplicated;
  }, [data]);

  // 테이블 데이터 처리 (CustomerPage 방식으로 통일)
  const processedTables = useMemo(() => {
    // 테이블이 배열이 아닌 경우 빈 배열 반환
    if (!data.tables || !Array.isArray(data.tables.tables)) return [];

    const tables = data.tables.tables.map((t) => ({
      id: t['Table.id'],
      max_customer: t['Table.max_customer'],
      status: t['Table.status'],
      // 데이터베이스의 좌표와 크기 값 명시적으로 숫자형으로 변환
      x: t['Table.x'] !== undefined ? Number(t['Table.x']) : undefined,
      y: t['Table.y'] !== undefined ? Number(t['Table.y']) : undefined,
      width: t['Table.width'] !== undefined ? Number(t['Table.width']) : undefined,
      height: t['Table.height'] !== undefined ? Number(t['Table.height']) : undefined,
    }));

    return tables.map((table) => {
      const hasAssignment = processedAssignmentsData.some((a) => a.table_id === table.id);
      return {
        ...table,
        status: hasAssignment ? 'OCCUPIED' : table.status || 'AVAILABLE',
        customer_id: hasAssignment 
          ? processedAssignmentsData.find((a) => a.table_id === table.id)?.customer_id 
          : null,
        // 데이터베이스 값 우선, 없는 경우에만 기본 위치 사용
        position: (table.x !== undefined && table.y !== undefined) 
          ? { x: table.x, y: table.y } 
          : TABLE_POSITIONS[table.id] || { x: 0, y: 0 }
      };
    });
  }, [data, processedAssignmentsData]);

  // 주문 데이터 처리
  const processedOrders = useMemo(() => {
    const rawOrders = orders && Array.isArray(orders.orders) ? orders.orders.flat() : [];
    return rawOrders.map(o => {
      const tsRaw = o['Order.timestamp'] || '';
      const timestamp = tsRaw.split('.')[0];
      const item = orders && Array.isArray(orders.orderitems)
        ? orders.orderitems.find(i => i['OrderItem.order_id'] === o['Order.id'])
        : null;
      const menu = orders && Array.isArray(orders.menuitems)
        ? orders.menuitems.find(m => m['MenuItem.id'] === item?.['OrderItem.menu_item_id'])
        : null;
      return {
        id: o['Order.id'], status: o['Order.status'], table: o['Order.table_id'],
        timestamp,
        items: item ? [{
          id: item['OrderItem.menu_item_id'],
          name: menu?.['MenuItem.name'] || 'Unknown',
          quantity: item['OrderItem.quantity'],
          price: menu?.['MenuItem.price'] || 0
        }] : [],
        totalPrice: item ? item['OrderItem.quantity'] * (menu?.['MenuItem.price'] || 0) : 0,
        totalQuantity: item ? item['OrderItem.quantity'] : 0,
      };
    });
  }, [orders]);

  const recentOrders = useMemo(() => {
    return [...processedOrders]
      .sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp))
      .slice(0, 10);
  }, [processedOrders]);

  // 이벤트 데이터 처리
  const processedEvents = useMemo(() => {
    const list = Array.isArray(events) ? events : [];
    return list.map(e => ({
      id: e['Event.id'], type: e['Event.type'],
      message: e['Event.description'] || '이벤트 발생',
      timestamp: e['Event.timestamp'],
      related_entity_type: e['Event.related_entity_type'],
      related_entity_id: e['Event.related_entity_id']
    }));
  }, [events]);
  const recentEvents = useMemo(() => {
    return [...processedEvents]
      .sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp))
      .slice(0, 10);
  }, [processedEvents]);

  // 시스템 로그 데이터 처리
  const processedLogs = useMemo(() => {
    // systemlogs가 배열인지 확인
    if (!Array.isArray(systemlogs)) {
      console.log('systemlogs가 배열이 아님:', systemlogs);
      return [];
    }

    // 새로운 형식 (WebSocketContext에서 추가한 테스트 데이터)
    if (systemlogs.length > 0 && systemlogs[0]?.id !== undefined && !systemlogs[0]['SystemLog.id']) {
      return systemlogs.map((log, index) => ({
        id: log.id || `log-${index}-${Date.now()}`,
        level: log.level,
        message: log.message,
        timestamp: log.timestamp,
        source: log.source
      }));
    }

    // 기존 형식
    return systemlogs.map((l, index) => ({
      id: l['SystemLog.id'] || l.id || `log-${index}-${Date.now()}`,
      level: l['SystemLog.level'] || l.level,
      message: l['SystemLog.message'] || l.message,
      timestamp: l['SystemLog.timestamp'] || l.timestamp,
      source: l['SystemLog.source'] || l.source
    }));
  }, [systemlogs]);
  
  const recentLogs = useMemo(() => {
    return [...processedLogs]
      .sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp))
      .slice(0, 30);
  }, [processedLogs]);

  // 수동 새로고침 핸들러
  const handleRefreshData = useCallback(() => {
    setIsLoading(true);
    TOPICS.forEach(topic => refreshTopic(topic));
    setTimeout(() => setIsLoading(false), 2000);
  }, [refreshTopic]);

  // 업데이트 시 로딩 해제 및 타임스탬프 갱신
  useEffect(() => {
    setLastUpdateTime(new Date());
    setIsLoading(false);
  }, [robots, tables, events, orders, status, systemlogs]);

  // 주문 상세 정보 보기 함수
  const handleViewOrder = useCallback((orderId) => {
    setSelectedOrderId(orderId);
    setIsOrderDetailModalOpen(true);
  }, []);

  // 주문 상세 정보 모달 닫기
  const handleCloseOrderModal = useCallback(() => {
    setIsOrderDetailModalOpen(false);
    setSelectedOrderId(null);
  }, []);

  return (
    <Layout>
      <div className="h-full w-full flex flex-col">
        <header className="flex-shrink-0 px-4 py-2 bg-white shadow-md flex items-center justify-between">
          <h1 className="text-2xl font-bold text-gray-800">대시보드</h1>
          <div className="flex items-center space-x-4">
            <p className="text-sm text-gray-500">마지막 업데이트: {formatDateTime(lastUpdateTime)}</p>
            <button
              onClick={handleRefreshData}
              className="p-2 bg-white border border-gray-300 rounded-md hover:bg-gray-50"
              disabled={isLoading}
            >
              <RefreshCw size={16} className={isLoading ? 'animate-spin' : ''} />
            </button>
          </div>
        </header>
        <main className="flex-1 grid grid-cols-12 grid-rows-[32rem_1fr] gap-4 p-4 h-full overflow-hidden">
          <section className="col-span-12 lg:col-span-8 h-full">
            <StoreMap
              tables={processedTables}
              assignments={processedAssignmentsData}
              robots={processedRobots.map(r => ({ ...r, latency: poseLatencies[r.id] }))}
              isLoading={isLoading}
              wsConnected={connected.robots}
              tablesError={errors.tables}
              robotsError={errors.robots}
            />
          </section>
          <section className="col-span-12 lg:col-span-4 flex flex-col min-h-0">
            <RobotStatusPanel
              className="h-full"
              robots={processedRobots}
              error={errors.robots}
              isLoading={isLoading}
            />
          </section>
          <section className="col-span-12 lg:col-span-6 flex flex-col min-h-0">
            <EventTimeline
              className="flex-1 overflow-auto"
              logs={recentLogs}
              error={errors.systemlogs}
              isLoading={isLoading}
            />
          </section>
          <section className="col-span-12 lg:col-span-6 flex flex-col min-h-0">
            <div className="flex-1 overflow-y-auto">
              <RecentOrders 
                orders={recentOrders} 
                onViewOrder={handleViewOrder}
              />
            </div>
          </section>
        </main>
      </div>

      {/* 주문 상세 정보 모달 */}
      {isOrderDetailModalOpen && selectedOrderId && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl max-w-2xl w-full mx-4 max-h-[90vh] overflow-y-auto">
            <div className="border-b px-6 py-4 flex items-center justify-between">
              <h2 className="text-xl font-semibold text-gray-800">
                주문 #{selectedOrderId} 상세 정보
              </h2>
              <button 
                onClick={handleCloseOrderModal}
                className="text-gray-400 hover:text-gray-500"
              >
                <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <line x1="18" y1="6" x2="6" y2="18"></line>
                  <line x1="6" y1="6" x2="18" y2="18"></line>
                </svg>
              </button>
            </div>
            
            <div className="px-6 py-4">
              {processedOrders.find(order => order.id === selectedOrderId) ? (
                <div>
                  <div className="mb-4">
                    <p className="text-sm text-gray-500 mb-1">테이블 번호</p>
                    <p className="text-lg font-medium">
                      테이블 {processedOrders.find(order => order.id === selectedOrderId).table || '정보 없음'}
                    </p>
                  </div>
                  <div className="mb-4">
                    <p className="text-sm text-gray-500 mb-1">주문 시간</p>
                    <p className="text-lg font-medium">
                      {new Date(processedOrders.find(order => order.id === selectedOrderId).timestamp).toLocaleString('ko-KR')}
                    </p>
                  </div>
                  <div className="mb-4">
                    <p className="text-sm text-gray-500 mb-1">주문 상태</p>
                    <p className="text-lg font-medium">
                      {processedOrders.find(order => order.id === selectedOrderId).status}
                    </p>
                  </div>
                  
                  <div className="mb-4">
                    <h3 className="text-lg font-medium border-b pb-2 mb-3">주문 항목</h3>
                    <div className="bg-gray-50 rounded-lg p-4">
                      <table className="min-w-full">
                        <thead>
                          <tr>
                            <th className="pb-2 text-left text-sm font-semibold text-gray-500">메뉴 이름</th>
                            <th className="pb-2 text-center text-sm font-semibold text-gray-500">수량</th>
                            <th className="pb-2 text-right text-sm font-semibold text-gray-500">가격</th>
                          </tr>
                        </thead>
                        <tbody>
                          {processedOrders.find(order => order.id === selectedOrderId).items.map((item, index) => (
                            <tr key={index} className="border-t border-gray-200">
                              <td className="py-2 text-sm">{item.name}</td>
                              <td className="py-2 text-sm text-center">{item.quantity}개</td>
                              <td className="py-2 text-sm text-right">{(item.price * item.quantity).toLocaleString()}원</td>
                            </tr>
                          ))}
                          <tr className="border-t border-gray-200 font-bold">
                            <td className="pt-3 text-sm">총계</td>
                            <td className="pt-3 text-sm text-center">{processedOrders.find(order => order.id === selectedOrderId).totalQuantity}개</td>
                            <td className="pt-3 text-sm text-right">{processedOrders.find(order => order.id === selectedOrderId).totalPrice.toLocaleString()}원</td>
                          </tr>
                        </tbody>
                      </table>
                    </div>
                  </div>
                </div>
              ) : (
                <div className="text-center py-8 text-gray-500">
                  <p>주문 정보를 찾을 수 없습니다.</p>
                </div>
              )}
            </div>
          </div>
        </div>
      )}
    </Layout>
  );
}
