import React, { useState, useRef, useEffect } from 'react';
import { ZoomIn, ZoomOut, RefreshCw } from 'react-feather';

const TableMap = ({ tables, assignments, onTableClick, selectedCustomer }) => {
  // 식당 영역 좌표
  const restaurantArea = {
    topLeft: { x: 21, y: 51 },
    topRight: { x: 62, y: 51 },
    bottomLeft: { x: 21, y: 75 },
    bottomRight: { x: 62, y: 75 }
  };

  // 식당 중심점 계산
  const centerX = (restaurantArea.topLeft.x + restaurantArea.topRight.x) / 2;
  const centerY = (restaurantArea.topLeft.y + restaurantArea.bottomLeft.y) / 2;

  // 기본 확대/축소 비율
  const [scale, setScale] = useState(15);
  
  // 기본 위치 - 식당 영역이 중앙에 오도록 조정
  const [position, setPosition] = useState(null);
  
  const [isDragging, setIsDragging] = useState(false);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const mapContainerRef = useRef(null);
  
  // 맵 크기 설정
  const MAP_WIDTH = 99;
  const MAP_HEIGHT = 100;

  // 테이블 시각화 크기 설정
  const TABLE_SIZE = 4;

  // 컴포넌트 마운트 시 식당 영역이 중앙에 오도록 위치 초기화
  useEffect(() => {
    if (!mapContainerRef.current) return;
    
    const containerWidth = mapContainerRef.current.clientWidth;
    const containerHeight = mapContainerRef.current.clientHeight;
    setPosition({ 
      x: containerWidth/2 - centerX*scale, 
      y: containerHeight/2 - centerY*scale 
    });
  }, [centerX, centerY, scale]);

  const zoomIn = () => {
    setScale(prev => Math.min(prev + 2, 25));
  };

  const zoomOut = () => {
    setScale(prev => Math.max(prev - 2, 10));
  };

  const resetZoom = () => {
    const newScale = 15;
    setScale(newScale);
    
    if (mapContainerRef.current) {
      const containerWidth = mapContainerRef.current.clientWidth;
      const containerHeight = mapContainerRef.current.clientHeight;
      setPosition({ 
        x: containerWidth/2 - centerX*newScale, 
        y: containerHeight/2 - centerY*newScale 
      });
    }
  };

  // 드래그 시작 핸들러
  const handleMouseDown = (e) => {
    setIsDragging(true);
    const currX = position?.x || 0;
    const currY = position?.y || 0;
    setDragStart({
      x: e.clientX - currX,
      y: e.clientY - currY
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

  // 테이블 클릭 처리 함수
  const handleTableClick = (tableId) => {
    if (onTableClick) {
      onTableClick(tableId);
    }
  };

  // 테이블의 배정 상태 확인
  const getTableStatus = (tableId) => {
    const assignment = assignments.find(a => a.table_id === tableId);
    if (assignment) {
      return 'OCCUPIED';
    }
    return 'AVAILABLE';
  };
  
  // 테이블 할당 가능 여부 확인 (선택된 고객이 있는 경우)
  const isAssignable = (tableId) => {
    // 선택된 고객이 없으면 할당 불가
    if (!selectedCustomer) return false;
    
    // 해당 테이블에 이미 할당된 고객이 있으면 할당 불가
    const assignment = assignments.find(a => a.table_id === tableId);
    if (assignment) return false;
    
    // 그 외의 경우 할당 가능
    return true;
  };

  // 테이블 상태에 따른 색상 클래스
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

  // 테이블 위치 (좌표 계산)
  const getTablePosition = (table, index) => {
    // 테이블 위치 정보가 이미 있는 경우 사용
    if (table.x && table.y) {
      return { x: table.x, y: table.y };
    }
    
    // 식당 내 테이블 위치 계산 (2x2 그리드로 배치)
    const tableSpacingX = (restaurantArea.topRight.x - restaurantArea.topLeft.x) / 3;
    const tableSpacingY = (restaurantArea.bottomLeft.y - restaurantArea.topLeft.y) / 3;
    
    // 테이블 번호에 따라 위치 계산
    const tableNum = table.id || (index + 1);
    
    switch (tableNum % 4) {
      case 1:
        return {
          x: restaurantArea.topLeft.x + tableSpacingX,
          y: restaurantArea.topLeft.y + tableSpacingY
        };
      case 2:
        return {
          x: restaurantArea.topRight.x - tableSpacingX,
          y: restaurantArea.topLeft.y + tableSpacingY
        };
      case 3:
        return {
          x: restaurantArea.topLeft.x + tableSpacingX,
          y: restaurantArea.bottomLeft.y - tableSpacingY
        };
      case 0:
        return {
          x: restaurantArea.topRight.x - tableSpacingX,
          y: restaurantArea.bottomLeft.y - tableSpacingY
        };
      default:
        return {
          x: centerX,
          y: centerY
        };
    }
  };

  return (
    <div className="w-full md:w-2/3 bg-white rounded-lg shadow ml-0 md:ml-4 mt-4 md:mt-0">
      <div className="p-4 border-b flex justify-between items-center">
        <h2 className="text-lg font-semibold flex items-center">
          테이블 맵
        </h2>
        <div className="flex space-x-2">
          <button
            onClick={zoomIn}
            className="p-1 rounded-full bg-gray-100 text-gray-700 hover:bg-gray-200"
            title="확대"
          >
            <ZoomIn size={16} />
          </button>
          <button
            onClick={zoomOut}
            className="p-1 rounded-full bg-gray-100 text-gray-700 hover:bg-gray-200"
            title="축소"
          >
            <ZoomOut size={16} />
          </button>
          <button
            onClick={resetZoom}
            className="p-1 rounded-full bg-gray-100 text-gray-700 hover:bg-gray-200"
            title="초기화"
          >
            <RefreshCw size={16} />
          </button>
        </div>
      </div>
      
      <div className="p-4">
        {selectedCustomer && (
          <div className="mb-4 bg-blue-50 p-3 rounded-lg flex items-center">
            <span>
              <strong>{selectedCustomer.count}명</strong> 손님을 배정할 테이블을 선택하세요
            </span>
          </div>
        )}
        
        <div
          ref={mapContainerRef}
          className="relative border border-gray-300 rounded-lg overflow-hidden bg-gray-50"
          style={{ height: '400px', cursor: isDragging ? 'grabbing' : 'grab' }}
          onMouseDown={handleMouseDown}
        >
          <div
            className="absolute inset-0"
            style={{
              visibility: position ? 'visible' : 'hidden',
              transformOrigin: '0 0',
              transform: position 
                ? `scale(${scale}) translate(${position.x/scale}px, ${position.y/scale}px)` 
                : undefined,
              transition: isDragging ? 'none' : 'transform 0.3s ease',
              width: `${MAP_WIDTH}px`,
              height: `${MAP_HEIGHT}px`
            }}
          >
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
            {tables.map((table, index) => {
              // 테이블 상태 확인 (웹소켓이나 할당 정보 기반)
              const status = table.status || getTableStatus(table.id);
              // 할당 가능 여부
              const assignable = isAssignable(table.id);
              
              // 테이블 색상 결정
              let bgColorClass = getTableColor(status);
              if (assignable) {
                bgColorClass = 'bg-yellow-200 border-yellow-500'; // 할당 가능: 연한 노랑
              }
              
              // 테이블 위치 및 크기 정보
              const position = getTablePosition(table, index);
              
              return (
                <div
                  key={`table-${table.id}`}
                  className={`absolute border-[1px] cursor-pointer flex items-center justify-center transition-all ${bgColorClass}`}
                  style={{
                    left: `${position.x}px`,
                    top: `${position.y}px`,
                    width: `${TABLE_SIZE}px`,
                    height: `${TABLE_SIZE}px`,
                    transform: 'translate(-50%, -50%)',
                  }}
                  onClick={() => handleTableClick(table.id)}
                  title={`테이블 ${table.id} - ${status === 'OCCUPIED' ? '사용 중 (클릭 시 해제)' : '사용 가능'}`}
                >
                  <span className="absolute font-semibold text-[3px] text-gray-900 text-center" style={{ lineHeight: '1px' }}>{table.id}</span>
                </div>
              );
            })}
          </div>

          {/* 조작 힌트 */}
          <div className="absolute top-2 right-2 bg-blue-50 text-blue-700 text-xs px-2 py-1 rounded flex items-center">
            <span>드래그하여 이동, 확대/축소 가능</span>
          </div>
        </div>

        <div className="mt-3 text-sm text-gray-500">
          <div className="flex flex-wrap items-center gap-2">
            <div className="flex items-center">
              <div className="w-3 h-3 mr-1 bg-green-200 border border-green-500"></div>
              <span>이용 가능</span>
            </div>
            <div className="flex items-center">
              <div className="w-3 h-3 mr-1 bg-red-200 border border-red-500"></div>
              <span>이용 중 (클릭 시 해제)</span>
            </div>
            <div className="flex items-center">
              <div className="w-3 h-3 mr-1 bg-yellow-200 border border-yellow-500"></div>
              <span>선택된 고객 배정 가능</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default TableMap; 