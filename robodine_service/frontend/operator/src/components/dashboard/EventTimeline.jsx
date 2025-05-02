import React, { useState } from 'react';
import { Filter, Clock, AlertTriangle, AlertOctagon, AlertCircle, Server, PlusCircle } from 'react-feather';

// EventTimeline 컴포넌트 - 최근 이벤트 타임라인 표시
const EventTimeline = ({ events = [], onSelectEvent, error, isLoading, className = ''}) => {
  const [showFilterMenu, setShowFilterMenu] = useState(false);
  const [activeFilters, setActiveFilters] = useState(['ALL']);

  // 이벤트 필터링 함수
  const filterEvents = () => {
    if (!events || events.length === 0) return [];
    
    if (activeFilters.includes('ALL')) {
      return events;
    }
    
    return events.filter(event => 
      activeFilters.some(filter => event.type?.toUpperCase().includes(filter))
    );
  };

  // 필터 토글 함수
  const toggleFilter = (filterType) => {
    if (filterType === 'ALL') {
      setActiveFilters(['ALL']);
      return;
    }
    
    // 현재 ALL 필터가 활성화된 경우, 제거하고 새 필터만 추가
    if (activeFilters.includes('ALL')) {
      setActiveFilters([filterType]);
      return;
    }
    
    // 이미 필터가 있으면 제거, 없으면 추가
    if (activeFilters.includes(filterType)) {
      const newFilters = activeFilters.filter(f => f !== filterType);
      // 필터가 비어있으면 ALL 추가
      setActiveFilters(newFilters.length ? newFilters : ['ALL']);
    } else {
      setActiveFilters([...activeFilters, filterType]);
    }
  };
  // WELCOME = "WELCOME"
  // CALL = "CALL"
  // BIRTHDAY = "BIRTHDAY"
  // EMERGENCY = "EMERGENCY"
  // CLEANING = "CLEANING"
  // 이벤트 아이콘 가져오기
  const getEventIcon = (eventType) => {
    switch (eventType?.toUpperCase()) {
      case 'CLEANING':
        return <AlertTriangle className="text-yellow-500" size={18} />;
      case 'EMERGENCY':
        return <AlertOctagon className="text-red-500" size={18} />;
      case 'WELCOME':
        return <Server className="text-blue-500" size={18} />;
      case 'BIRTHDAY':
        return <Server className="text-purple-500" size={18} />;
      default:
        return <PlusCircle className="text-gray-500" size={18} />;
    }
  };

  // 이벤트 색상 가져오기
  const getEventColor = (eventType) => {
    switch (eventType?.toUpperCase()) {
      case 'CLEANING':
        return 'border-yellow-500';
      case 'EMERGENCY':
        return 'border-red-500';
      case 'WELCOME':
        return 'border-blue-500';
      case 'BIRTHDAY':
        return 'border-purple-500';
      default:
        return 'border-gray-300';
    }
  };

  // 타임스탬프 포맷팅
  const formatTimestamp = (timestamp) => {
    if (!timestamp) return '';
    const date = new Date(timestamp);
    return date.toLocaleString('ko-KR', { 
      month: 'short', 
      day: 'numeric', 
      hour: '2-digit', 
      minute: '2-digit',
      hour12: true
    });
  };

  // 필터링된 이벤트
  const filteredEvents = filterEvents();

  return (
    <div className={`bg-white … p-4 flex flex-col ${className}`}>
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-lg font-semibold text-gray-700 flex items-center">
          <Clock className="mr-2 text-blue-500" />
          최근 이벤트
        </h2>
        <div className="relative">
          <button 
            className="p-2 rounded-full hover:bg-gray-100 focus:outline-none"
            onClick={() => setShowFilterMenu(!showFilterMenu)}
          >
            <Filter size={18} className="text-gray-600" />
          </button>
          
          {showFilterMenu && (
            <div className="absolute right-0 mt-2 w-48 bg-white rounded-md shadow-lg z-10">
              <div className="py-1">
                <button 
                  className={`w-full text-left px-4 py-2 text-sm ${
                    activeFilters.includes('ALL') ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
                  }`}
                  onClick={() => toggleFilter('ALL')}
                >
                  모든 이벤트
                </button>
                <button 
                  className={`w-full text-left px-4 py-2 text-sm ${
                    activeFilters.includes('WARNING') ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
                  }`}
                  onClick={() => toggleFilter('WARNING')}
                >
                  경고
                </button>
                <button 
                  className={`w-full text-left px-4 py-2 text-sm ${
                    activeFilters.includes('ERROR') ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
                  }`}
                  onClick={() => toggleFilter('ERROR')}
                >
                  오류
                </button>
                <button 
                  className={`w-full text-left px-4 py-2 text-sm ${
                    activeFilters.includes('SYSTEM') ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
                  }`}
                  onClick={() => toggleFilter('SYSTEM')}
                >
                  시스템
                </button>
                <button 
                  className={`w-full text-left px-4 py-2 text-sm ${
                    activeFilters.includes('ROBOT') ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
                  }`}
                  onClick={() => toggleFilter('ROBOT')}
                >
                  로봇
                </button>
              </div>
            </div>
          )}
        </div>
      </div>
      
      {isLoading ? (
        <div className="flex items-center justify-center h-64">
          <div className="animate-spin rounded-full h-8 w-8 border-t-2 border-b-2 border-blue-500"></div>
        </div>
      ) : error ? (
        <div className="bg-red-50 text-red-600 p-4 rounded-md flex items-center">
          <AlertOctagon className="mr-2" size={20} />
          <span>{error}</span>
        </div>
      ) : filteredEvents.length === 0 ? (
        <div className="text-center py-10">
          <Clock className="mx-auto h-10 w-10 text-gray-400" />
          <h3 className="mt-2 text-sm font-medium text-gray-900">이벤트 없음</h3>
          <p className="mt-1 text-sm text-gray-500">
            {activeFilters.includes('ALL') 
              ? '아직 기록된 이벤트가 없습니다.' 
              : '선택한 필터에 해당하는 이벤트가 없습니다.'}
          </p>
        </div>
      ) : (
        // 이벤트 목록
        <div className="space-y-4 overflow-y-auto pr-2 flex-1">
          {filteredEvents.map((event) => (
            <div 
              key={event.id} 
              className={`border-l-4 ${getEventColor(event.type)} pl-3 py-2 hover:bg-gray-50 rounded cursor-pointer`}
              onClick={() => onSelectEvent && onSelectEvent(event)}
            >
              <div className="flex justify-between items-start">
                <div className="flex items-start">
                  <span className="mr-2 mt-0.5">
                    {getEventIcon(event.type)}
                  </span>
                  <div>
                    <p className="text-sm font-medium text-gray-900">{event.message}</p>
                    {event.robotId && (
                      <p className="text-xs text-gray-500">로봇 ID: {event.robotId}</p>
                    )}
                  </div>
                </div>
                <span className="text-xs text-gray-500 whitespace-nowrap ml-2">
                  {formatTimestamp(event.timestamp)}
                </span>
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default EventTimeline; 