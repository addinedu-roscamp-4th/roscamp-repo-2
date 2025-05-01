import React, { useState, useRef, useEffect } from 'react';
import { ZoomIn, ZoomOut, RefreshCw, AlertTriangle, Wifi } from 'react-feather';

const StoreMap = ({ tables = [], robots = [], isLoading, robotsError, tablesError, selectedEvent, wsConnected }) => {
  // 식당 영역 좌표 (최상단에 정의하여 다른 계산에 사용)
  const restaurantArea = {
    topLeft: { x: 21, y: 51 },
    topRight: { x: 62, y: 51 },
    bottomLeft: { x: 21, y: 75 },
    bottomRight: { x: 62, y: 75 }
  };

  // 식당 중심점 계산
  const centerX = (restaurantArea.topLeft.x + restaurantArea.topRight.x) / 2;
  const centerY = (restaurantArea.topLeft.y + restaurantArea.bottomLeft.y) / 2;

  // 기본 확대/축소 비율 - 식당 영역이 더 크게 보이도록 조정
  const [scale, setScale] = useState(15); 
  
  // 기본 위치 - 식당 영역이 중앙에 오도록 조정
  const [position, setPosition] = useState({ 
    x: 0,  // 초기 위치는 컴포넌트 마운트 후 설정
    y: 0
  });
  
  const [isDragging, setIsDragging] = useState(false);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const mapContainerRef = useRef(null);
  
  // 맵 크기 설정 - 좌표와 픽셀이 1:1 매핑
  const MAP_WIDTH = 99;  // 픽셀
  const MAP_HEIGHT = 100; // 픽셀

  // 컴포넌트 마운트 시 식당 영역이 중앙에 오도록 위치 초기화
  useEffect(() => {
    // 컨테이너 크기 확인
    if (mapContainerRef.current) {
      const containerWidth = mapContainerRef.current.clientWidth;
      const containerHeight = mapContainerRef.current.clientHeight;
      
      // 식당 중심점이 뷰포트 중앙에 오도록 조정
      const offsetX = (containerWidth / 2 / scale) - centerX;
      const offsetY = (containerHeight / 2 / scale) - centerY;
      
      setPosition({ x: offsetX * scale, y: offsetY * scale });
    }
  }, [scale, centerX, centerY]); // 의존성 배열에 centerX, centerY 추가

  const zoomIn = () => {
    setScale(prev => Math.min(prev + 0.5, 15)); // 최대 확대 레벨 증가
  };

  const zoomOut = () => {
    setScale(prev => Math.max(prev - 0.5, 1));
  };

  const resetZoom = () => {
    setScale(15); // 기본 확대 레벨
    
    // 식당 중심점이 뷰포트 중앙에 오도록 위치 재설정
    if (mapContainerRef.current) {
      const containerWidth = mapContainerRef.current.clientWidth;
      const containerHeight = mapContainerRef.current.clientHeight;
      
      const offsetX = (containerWidth / 2 / 15) - centerX;
      const offsetY = (containerHeight / 2 / 15) - centerY;
      
      setPosition({ x: offsetX * 15, y: offsetY * 15 });
    }
  };

  // 드래그 시작 핸들러
  const handleMouseDown = (e) => {
    setIsDragging(true);
    setDragStart({
      x: e.clientX - position.x,
      y: e.clientY - position.y
    });
    e.preventDefault();
  };

  // 드래그 중 핸들러
  const handleMouseMove = (e) => {
    if (isDragging) {
      setPosition({
        x: e.clientX - dragStart.x,
        y: e.clientY - dragStart.y
      });
      e.preventDefault();
    }
  };

  // 드래그 종료 핸들러
  const handleMouseUp = () => {
    setIsDragging(false);
  };

  // 드래그 이벤트 등록 및 제거
  useEffect(() => {
    const containerElement = mapContainerRef.current;
    
    if (containerElement) {
      containerElement.addEventListener('mousemove', handleMouseMove);
      containerElement.addEventListener('mouseup', handleMouseUp);
      containerElement.addEventListener('mouseleave', handleMouseUp);
    }
    
    return () => {
      if (containerElement) {
        containerElement.removeEventListener('mousemove', handleMouseMove);
        containerElement.removeEventListener('mouseup', handleMouseUp);
        containerElement.removeEventListener('mouseleave', handleMouseUp);
      }
    };
  }, [isDragging, dragStart]);

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
    switch (status?.toUpperCase()) {
      case 'ONLINE':
      case 'IDLE':
        return 'bg-green-400';
      case 'BUSY':
        return 'bg-blue-400';
      case 'WARNING':
        return 'bg-yellow-400';
      case 'OFFLINE':
      case 'NOT_CONNECTED':
        return 'bg-red-400';
      default:
        return 'bg-gray-400';
    }
  };

  // 각 항목에 기본 위치 할당
  const getDefaultPosition = (id, type) => {
    // 식당 영역 내에 위치하도록 조정
    const minX = restaurantArea.topLeft.x + 5;
    const maxX = restaurantArea.topRight.x - 5;
    const minY = restaurantArea.topLeft.y + 5;
    const maxY = restaurantArea.bottomLeft.y - 5;
    
    const width = maxX - minX;
    const height = maxY - minY;
    
    const padding = 3;
    
    // 테이블과 로봇이 겹치지 않도록 기본 위치 분산 (식당 영역 내)
    const baseX = type === 'table' 
      ? minX + padding + ((id % 4) * (width / 4))
      : minX + padding + ((id % 3) * (width / 3));
      
    const baseY = type === 'table' 
      ? minY + padding + (Math.floor(id / 4) * (height / 4))
      : minY + padding + (Math.floor(id / 3) * (height / 3));
      
    return { x: baseX, y: baseY };
  };

  // 항목이 하이라이트 되어야 하는지 확인
  const isHighlighted = (item) => {
    if (!selectedEvent || !selectedEvent.location || !item.position) return false;
    
    const itemX = item.position.x;
    const itemY = item.position.y;
    const eventX = selectedEvent.location.x;
    const eventY = selectedEvent.location.y;
    
    return Math.abs(itemX - eventX) < 2 && Math.abs(itemY - eventY) < 2;
  };

  // 에러 메시지 표시 여부 확인
  const hasError = robotsError || tablesError;
  
  // 맵에 표시할 유효한 테이블과 로봇 필터링
  const validTables = tables.filter(table => table && (table.position || table.id));
  const validRobots = robots.filter(robot => robot && (robot.position || robot.id));

  // 테이블 기본 위치 (고정) - 좌표 1:1 변환 (식당 영역 내 균등 분포)
  const tablePositions = {};
  
  // 식당 내 테이블 위치 계산 (2x2 그리드로 배치)
  const tableSpacingX = (restaurantArea.topRight.x - restaurantArea.topLeft.x) / 3;
  const tableSpacingY = (restaurantArea.bottomLeft.y - restaurantArea.topLeft.y) / 3;
  
  // 4개의 테이블만 배치
  tablePositions[1] = {
    x: restaurantArea.topLeft.x + tableSpacingX,
    y: restaurantArea.topLeft.y + tableSpacingY
  };
  tablePositions[2] = {
    x: restaurantArea.topRight.x - tableSpacingX,
    y: restaurantArea.topLeft.y + tableSpacingY
  };
  tablePositions[3] = {
    x: restaurantArea.topLeft.x + tableSpacingX,
    y: restaurantArea.bottomLeft.y - tableSpacingY
  };
  tablePositions[4] = {
    x: restaurantArea.topRight.x - tableSpacingX,
    y: restaurantArea.bottomLeft.y - tableSpacingY
  };

  // 테이블 및 로봇 시각화 크기 설정
  // TIP: 아래 값을 수정하여 테이블과 로봇의 크기를 조절할 수 있습니다.
  const TABLE_SIZE = 4; // 테이블 크기 (픽셀) - 식당 영역에 맞게 조정
  const ROBOT_SIZE = 3;  // 로봇 크기 (픽셀) - 더 크게 설정

  // 임시로 테이블 데이터 추가 (디버깅용)
  console.log("Tables:", validTables);
  const debugTables = Array.from({ length: 4 }, (_, i) => ({
    id: i + 1,
    tableNumber: i + 1,
    status: i % 3 === 0 ? 'AVAILABLE' : i % 3 === 1 ? 'OCCUPIED' : 'RESERVED',
    position: tablePositions[i + 1]
  }));

  return (
    <div className="bg-white rounded-lg shadow p-4 h-full">
      <div className="flex justify-between items-center mb-4">
        <div className="flex items-center">
          <h2 className="text-lg font-semibold text-gray-700">매장 맵</h2>
          {wsConnected && (
            <span className="ml-2 flex items-center text-xs text-green-600 bg-green-50 px-2 py-1 rounded-full">
              <Wifi size={12} className="mr-1" />
              연결됨
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
        <div 
          ref={mapContainerRef}
          className="relative border border-gray-300 rounded-lg overflow-hidden bg-gray-50" 
          style={{ height: '400px', cursor: isDragging ? 'grabbing' : 'grab' }}
          onMouseDown={handleMouseDown}
        >
          <div className="absolute inset-0" style={{ 
            transform: `scale(${scale}) translate(${position.x / scale}px, ${position.y / scale}px)`, 
            transformOrigin: 'center', 
            transition: isDragging ? 'none' : 'transform 0.3s ease',
            width: `${MAP_WIDTH}px`,
            height: `${MAP_HEIGHT}px`
          }}>
            {/* 격자 배경 */}
            <div className="absolute inset-0">
              {Array.from({ length: Math.floor(MAP_HEIGHT / 2) }).map((_, y) => (
                <div key={`grid-y-${y}`} className="absolute w-full h-px bg-gray-200" style={{ top: `${y * 2}px` }} />
              ))}
              {Array.from({ length: Math.floor(MAP_WIDTH / 2) }).map((_, x) => (
                <div key={`grid-x-${x}`} className="absolute w-px h-full bg-gray-200" style={{ left: `${x * 2}px` }} />
              ))}
            </div>
            
            {/* 식당 영역 테두리 */}
            <div 
              className="absolute border-[2px] border-blue-700 z-0 bg-blue-50 bg-opacity-30"
              style={{
                left: `${restaurantArea.topLeft.x}px`,
                top: `${restaurantArea.topLeft.y}px`,
                width: `${restaurantArea.topRight.x - restaurantArea.topLeft.x}px`,
                height: `${restaurantArea.bottomLeft.y - restaurantArea.topLeft.y}px`
              }}
            />
            
            {/* 테이블 */}
            {/* TIP: style 내의 width와 height를 수정하여 테이블 크기 조절 가능 */}
            {(validTables.length > 0 ? validTables : debugTables).map(table => {
              // 1~4번 테이블만 표시
              if (table.tableNumber > 4) return null;
              
              const position = table.position || tablePositions[table.tableNumber] || getDefaultPosition(table.id || 0, 'table');
              return (
                <div
                  key={`table-${table.id}`}
                  className={`absolute border-[0.1px] flex items-center justify-center transition-all ${getTableColor(table.status)} ${isHighlighted(table) ? 'ring-1 ring-blue-500 z-10' : ''}`}
                  style={{
                    left: `${position.x}px`,
                    top: `${position.y}px`,
                    width: `${TABLE_SIZE}px`,
                    height: `${TABLE_SIZE}px`,
                    transform: 'translate(-50%, -50%)',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center'
                  }}
                  title={`테이블 ${table.tableNumber || table.id} - ${table.status || '상태 없음'}`}
                >
                  <span className="absolute font-semibold text-[3px] text-gray-900 text-center" style={{ lineHeight: '1px' }}>{table.tableNumber || table.id}</span>
                </div>
              );
            })}
            
            {/* 로봇 */}
            {/* TIP: style 내의 width와 height를 수정하여 로봇 크기 조절 가능 */}
            {validRobots.map(robot => {
              const position = robot.position || getDefaultPosition(robot.id || 0, 'robot');
              return (
                <div
                  key={`robot-${robot.id}`}
                  className={`absolute rounded-full flex items-center justify-center ${getRobotColor(robot.status)} ${isHighlighted(robot) ? 'ring-1 ring-blue-500 z-20' : 'z-10'}`}
                  style={{
                    left: `${position.x}px`,
                    top: `${position.y}px`,
                    width: `${ROBOT_SIZE}px`,
                    height: `${ROBOT_SIZE}px`,
                    transform: 'translate(-50%, -50%)',
                    boxShadow: '0 0 0.5px rgba(0,0,0,0.3)'
                  }}
                  title={`${robot.name || `로봇 ${robot.id}`} - ${robot.status || '상태 없음'}`}
                >
                  <span className="absolute font-semibold text-[2px] text-black text-center" style={{ lineHeight: '1px' }}>{robot.id}</span>
                </div>
              );
            })}
            
            {/* 이벤트 하이라이트 */}
            {selectedEvent && selectedEvent.location && (
              <div
                className="absolute rounded-full border-2 border-dashed border-blue-500 z-0 animate-pulse"
                style={{
                  left: `${selectedEvent.location.x}px`,
                  top: `${selectedEvent.location.y}px`,
                  width: `${TABLE_SIZE / 2}px`,
                  height: `${TABLE_SIZE / 2}px`,
                  transform: 'translate(-50%, -50%)'
                }}
              />
            )}
          </div>
          
          {/* 웹소켓 연결 오류 메시지 */}
          {!wsConnected && (
            <div className="absolute bottom-2 right-2 bg-red-50 text-red-600 text-xs px-2 py-1 rounded flex items-center">
              {/* <AlertTriangle size={12} className="mr-1" />
              웹소켓 연결 오류 */}
            </div>
          )}
          
          {/* 조작 힌트 */}
          <div className="absolute top-2 right-2 bg-blue-50 text-blue-700 text-xs px-2 py-1 rounded flex items-center">
            <span>드래그하여 이동, 확대/축소 가능</span>
          </div>
        </div>
      )}
      
      <div className="mt-3 text-sm text-gray-500">
        <div className="flex flex-wrap items-center gap-2">
          <div className="flex items-center">
            <div className="w-3 h-3 mr-1 bg-green-200 border border-green-500"></div>
            <span>사용 가능</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 mr-1 bg-red-200 border border-red-500"></div>
            <span>사용 중</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 mr-1 bg-yellow-200 border border-yellow-500"></div>
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