import React from 'react';
import { AlertTriangle, Map, Navigation, Users } from 'react-feather';

const StoreMap = ({ tables, robots, selectedEvent, tablesError, robotsError }) => {
  // 두 가지 에러 모두 있는 경우
  if (tablesError && robotsError) {
    return (
      <div className="bg-white p-4 rounded-lg shadow">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">매장 지도</h2>
        <div className="bg-red-50 border border-red-200 text-red-700 px-4 py-8 rounded flex items-center justify-center">
          <AlertTriangle className="mr-2" size={20} />
          <p>테이블과 로봇 정보를 불러올 수 없습니다</p>
        </div>
      </div>
    );
  }

  // 테이블 혹은 로봇 데이터 없는 경우에도 지도는 표시
  const displayMap = () => {
    // 지도 컨테이너 크기
    const mapWidth = 800;
    const mapHeight = 600;

    // 테이블 상태에 따른 색상
    const getTableColor = (status) => {
      switch (status) {
        case 'OCCUPIED':
          return '#EF4444'; // 빨간색
        case 'AVAILABLE':
          return '#10B981'; // 초록색
        case 'RESERVED':
          return '#F59E0B'; // 노란색
        default:
          return '#9CA3AF'; // 회색
      }
    };

    // 로봇 상태에 따른 색상
    const getRobotColor = (status) => {
      switch (status) {
        case 'ONLINE':
          return '#3B82F6'; // 파란색
        case 'BUSY':
          return '#F59E0B'; // 노란색
        case 'ERROR':
          return '#EF4444'; // 빨간색
        default:
          return '#9CA3AF'; // 회색
      }
    };

    return (
      <div className="relative bg-gray-100 rounded-lg overflow-hidden" style={{ width: '100%', height: '500px' }}>
        {/* 지도 그리드 */}
        <div className="absolute inset-0" style={{ 
          backgroundSize: '40px 40px',
          backgroundImage: 'linear-gradient(to right, #e5e7eb 1px, transparent 1px), linear-gradient(to bottom, #e5e7eb 1px, transparent 1px)'
        }}></div>
        
        {/* 테이블 렌더링 */}
        {tables && tables.map((table) => (
          <div 
            key={`table-${table.id}`}
            className="absolute flex items-center justify-center rounded-full shadow-md transition-all"
            style={{ 
              left: `${table.position.x}px`, 
              top: `${table.position.y}px`,
              width: '60px',
              height: '60px',
              backgroundColor: getTableColor(table.status),
              transform: 'translate(-50%, -50%)'
            }}
          >
            <div className="flex flex-col items-center text-white">
              <Users size={16} />
              <span className="text-xs mt-1">{table.number}</span>
            </div>
          </div>
        ))}

        {/* 로봇 렌더링 */}
        {robots && robots.map((robot) => (
          <div 
            key={`robot-${robot.id}`}
            className="absolute flex items-center justify-center rounded-lg shadow-lg transition-all"
            style={{ 
              left: `${robot.position.x}px`, 
              top: `${robot.position.y}px`,
              width: '40px',
              height: '40px',
              backgroundColor: getRobotColor(robot.status),
              transform: 'translate(-50%, -50%)',
              zIndex: 10
            }}
          >
            <div className="flex flex-col items-center text-white">
              <Navigation size={16} />
              <span className="text-xs mt-1">{robot.name.substring(0, 1)}</span>
            </div>
          </div>
        ))}

        {/* 선택된 이벤트 위치 하이라이트 */}
        {selectedEvent && selectedEvent.location && (
          <div 
            className="absolute w-16 h-16 border-4 border-yellow-500 rounded-full animate-ping"
            style={{ 
              left: selectedEvent.location.x || 100,
              top: selectedEvent.location.y || 100,
              transform: 'translate(-50%, -50%)',
              zIndex: 20
            }}
          ></div>
        )}

        {/* 에러 오버레이 표시 */}
        {tablesError && (
          <div className="absolute bottom-0 left-0 right-0 bg-red-100 text-red-700 p-2 text-sm flex items-center">
            <AlertTriangle size={16} className="mr-1" />
            {tablesError}
          </div>
        )}
        {robotsError && !tablesError && (
          <div className="absolute bottom-0 left-0 right-0 bg-red-100 text-red-700 p-2 text-sm flex items-center">
            <AlertTriangle size={16} className="mr-1" />
            {robotsError}
          </div>
        )}
        
        {/* 지도 제어 UI */}
        <div className="absolute top-2 right-2 bg-white rounded-md shadow p-2 z-20">
          <Map size={20} className="text-gray-500" />
        </div>
      </div>
    );
  };

  return (
    <div className="bg-white p-4 rounded-lg shadow">
      <h2 className="text-lg font-semibold text-gray-800 mb-4">매장 지도</h2>
      {displayMap()}
    </div>
  );
};

export default StoreMap; 