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

  // 테이블 데이터 처리
  const processedTables = useMemo(() => {
    const list = Array.isArray(tables) ? tables : [];
    return list.map(t => ({
      id: t['Table.id'], tableNumber: t['Table.table_number'],
      status: t['Table.status'], maxCustomer: t['Table.max_customer'],
      updatedAt: t['Table.updated_at'],
      position: TABLE_POSITIONS[t['Table.table_number']] || { x: 0, y: 0 }
    }));
  }, [tables]);

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
        totalPrice: item ? item['OrderItem.quantity'] * (menu?.['MenuItem.price'] || 0) : 0
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
    const list = Array.isArray(systemlogs) ? systemlogs : [];
    return list.map(l => ({
      id: l['SystemLog.id'], level: l['SystemLog.level'], message: l['SystemLog.message'],
      timestamp: l['SystemLog.timestamp']
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
              robots={processedRobots.map(r => ({ ...r, latency: poseLatencies[r.id] }))}
              isLoading={isLoading}
              wsConnected={connected.robots}
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
              <RecentOrders orders={recentOrders} />
            </div>
          </section>
        </main>
      </div>
    </Layout>
  );
}
