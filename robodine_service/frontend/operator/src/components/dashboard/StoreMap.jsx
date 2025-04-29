import React, { useState } from 'react';
import { ZoomIn, ZoomOut, RefreshCw, AlertTriangle, Wifi } from 'react-feather';

const StoreMap = ({ tables = [], robots = [], isLoading, robotsError, tablesError, selectedEvent, wsConnected }) => {
  const [scale, setScale] = useState(1);

  const zoomIn = () => {
    setScale(prev => Math.min(prev + 0.1, 2));
  };

  const zoomOut = () => {
    setScale(prev => Math.max(prev - 0.1, 0.5));
  };

  const resetZoom = () => {
    setScale(1);
  };

  const getTableColor = (status) => {
    switch (status) {
      case 'OCCUPIED':
        return 'bg-red-200 border-red-500';
      case 'AVAILABLE':
        return 'bg-green-200 border-green-500';
      case 'RESERVED':
        return 'bg-yellow-200 border-yellow-500';
      default:
        return 'bg-gray-200 border-gray-500';
    }
  };

  const getRobotColor = (status) => {
    switch (status) {
      case 'ONLINE':
        return 'bg-green-400';
      case 'BUSY':
        return 'bg-blue-400';
      case 'WARNING':
        return 'bg-yellow-400';
      case 'OFFLINE':
        return 'bg-red-400';
      default:
        return 'bg-gray-400';
    }
  };

  // 각 항목에 기본 위치 할당
  const getDefaultPosition = (id, type) => {
    // 테이블과 로봇이 겹치지 않도록 기본 위치 분산
    const baseX = type === 'table' ? 100 + (id * 50) % 500 : 250 + (id * 60) % 400;
    const baseY = type === 'table' ? 100 + Math.floor(id / 10) * 70 : 200 + Math.floor(id / 8) * 80;
    return { x: baseX, y: baseY };
  };

  // 항목이 하이라이트 되어야 하는지 확인
  const isHighlighted = (item) => {
    if (!selectedEvent || !selectedEvent.location || !item.position) return false;
    
    const itemX = item.position.x;
    const itemY = item.position.y;
    const eventX = selectedEvent.location.x;
    const eventY = selectedEvent.location.y;
    
    return Math.abs(itemX - eventX) < 20 && Math.abs(itemY - eventY) < 20;
  };

  // 에러 메시지 표시 여부 확인
  const hasError = robotsError || tablesError;
  
  // 맵에 표시할 유효한 테이블과 로봇 필터링
  const validTables = tables.filter(table => table && (table.position || table.id));
  const validRobots = robots.filter(robot => robot && (robot.position || robot.id));

  return (
    <div className="bg-white rounded-lg shadow p-4 h-full">
      <div className="flex justify-between items-center mb-4">
        <div className="flex items-center">
          <h2 className="text-lg font-semibold text-gray-700">매장 맵</h2>
          {wsConnected && (
            <span className="ml-2 flex items-center text-xs text-yellow-600 bg-yellow-50 px-2 py-1 rounded-full">
              <Wifi size={12} className="mr-1" />
              연결됨 (데이터 수신 기능 미구현)
            </span>
          )}
        </div>
        <div className="flex space-x-2">
          <button
            onClick={zoomIn}
            className="p-2 rounded-full bg-gray-100 text-gray-700 hover:bg-gray-200"
            title="확대"
          >
            <ZoomIn size={16} />
          </button>
          <button
            onClick={zoomOut}
            className="p-2 rounded-full bg-gray-100 text-gray-700 hover:bg-gray-200"
            title="축소"
          >
            <ZoomOut size={16} />
          </button>
          <button
            onClick={resetZoom}
            className="p-2 rounded-full bg-gray-100 text-gray-700 hover:bg-gray-200"
            title="초기화"
          >
            <RefreshCw size={16} />
          </button>
        </div>
      </div>
      
      {isLoading ? (
        <div className="flex items-center justify-center h-64">
          <div className="animate-spin rounded-full h-8 w-8 border-t-2 border-b-2 border-blue-500"></div>
        </div>
      ) : hasError ? (
        <div className="bg-red-50 text-red-600 p-4 rounded-md flex flex-col items-center">
          <AlertTriangle className="mb-2" size={24} />
          <div>
            {robotsError && <p className="text-center mb-1">{robotsError}</p>}
            {tablesError && <p className="text-center">{tablesError}</p>}
          </div>
          <p className="text-center text-sm text-red-500 mt-2">
            맵 데이터를 가져올 수 없어 일부 정보만 표시됩니다
          </p>
        </div>
      ) : (
        <div className="relative border border-gray-300 rounded-lg overflow-hidden bg-gray-50" style={{ height: '400px' }}>
          <div className="absolute inset-0" style={{ transform: `scale(${scale})`, transformOrigin: 'center', transition: 'transform 0.3s ease' }}>
            {/* Tables */}
            {validTables.map(table => {
              const position = table.position || getDefaultPosition(table.id || 0, 'table');
              return (
                <div
                  key={`table-${table.id}`}
                  className={`absolute w-14 h-14 rounded-md border-2 flex items-center justify-center transition-all ${getTableColor(table.status)} ${isHighlighted(table) ? 'ring-4 ring-blue-500 z-10' : ''}`}
                  style={{
                    left: `${position.x}px`,
                    top: `${position.y}px`,
                    transform: 'translate(-50%, -50%)'
                  }}
                  title={`테이블 ${table.number || table.id} - ${table.status || '상태 없음'}`}
                >
                  <span className="font-bold text-gray-700">{table.number || table.id}</span>
                </div>
              );
            })}
            
            {/* Robots */}
            {validRobots.map(robot => {
              const position = robot.position || getDefaultPosition(robot.id || 0, 'robot');
              return (
                <div
                  key={`robot-${robot.id}`}
                  className={`absolute w-8 h-8 rounded-full border-2 border-gray-600 flex items-center justify-center ${getRobotColor(robot.status)} ${isHighlighted(robot) ? 'ring-4 ring-blue-500 z-10' : ''}`}
                  style={{
                    left: `${position.x}px`,
                    top: `${position.y}px`,
                    transform: 'translate(-50%, -50%)'
                  }}
                  title={`${robot.name || `로봇 ${robot.id}`} - ${robot.status || '상태 없음'}`}
                >
                  <span className="font-bold text-xs text-white">{robot.id}</span>
                </div>
              );
            })}
            
            {/* Event highlight */}
            {selectedEvent && selectedEvent.location && (
              <div
                className="absolute w-16 h-16 rounded-full border-4 border-dashed border-blue-500 z-0 animate-pulse"
                style={{
                  left: `${selectedEvent.location.x}px`,
                  top: `${selectedEvent.location.y}px`,
                  transform: 'translate(-50%, -50%)'
                }}
              />
            )}
          </div>
          
          {/* 실시간 업데이트 정보 */}
          <div className="absolute bottom-2 right-2 bg-yellow-50 text-yellow-700 text-xs px-2 py-1 rounded flex items-center">
            <Wifi size={12} className="mr-1" />
            {wsConnected ? 
              '실시간 위치 업데이트 기능이 구현되지 않았습니다' : 
              '웹소켓 연결되지 않음'
            }
          </div>
          
          {/* 정적 위치 정보 알림 */}
          <div className="absolute top-2 left-2 bg-blue-50 text-blue-700 text-xs px-2 py-1 rounded flex items-center">
            <AlertTriangle size={12} className="mr-1" />
            위치 정보는 정적 데이터입니다
          </div>
        </div>
      )}
      
      <div className="mt-3 text-sm text-gray-500">
        <div className="flex flex-wrap items-center gap-2">
          <div className="flex items-center">
            <div className="w-3 h-3 mr-1 rounded-sm bg-green-200 border border-green-500"></div>
            <span>사용 가능</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 mr-1 rounded-sm bg-red-200 border border-red-500"></div>
            <span>사용 중</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 mr-1 rounded-sm bg-yellow-200 border border-yellow-500"></div>
            <span>예약됨</span>
          </div>
          <div className="flex items-center ml-4">
            <div className="w-3 h-3 mr-1 rounded-full bg-green-400"></div>
            <span>로봇 온라인</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 mr-1 rounded-full bg-red-400"></div>
            <span>로봇 오프라인</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default StoreMap; 