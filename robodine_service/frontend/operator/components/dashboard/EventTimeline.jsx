import React, { useState } from 'react';
import { 
  AlertCircle, AlertTriangle, Info, 
  Clock, Server, Shield, ChevronDown, AlertOctagon 
} from 'react-feather';

const EventTimeline = ({ events, onSelectEvent, error }) => {
  const [filterMenuOpen, setFilterMenuOpen] = useState(false);
  const [filters, setFilters] = useState({
    WARNING: true,
    ERROR: true,
    SYSTEM: true,
    ROBOT: true,
    EMERGENCY: true,
  });

  // 필터 토글 함수
  const toggleFilter = (type) => {
    setFilters({
      ...filters,
      [type]: !filters[type],
    });
  };

  // 이벤트 필터링 함수
  const filteredEvents = events && events.filter(event => filters[event.type]);

  // 이벤트 타입별 아이콘 가져오기
  const getEventIcon = (type) => {
    switch (type) {
      case 'WARNING':
        return <AlertTriangle size={18} className="text-yellow-500" />;
      case 'ERROR':
        return <AlertCircle size={18} className="text-red-500" />;
      case 'SYSTEM':
        return <Server size={18} className="text-blue-500" />;
      case 'ROBOT':
        return <Clock size={18} className="text-green-500" />;
      case 'EMERGENCY':
        return <AlertOctagon size={18} className="text-red-600" />;
      default:
        return <Info size={18} className="text-gray-500" />;
    }
  };

  // 이벤트 타입별 색상 가져오기
  const getEventColor = (type) => {
    switch (type) {
      case 'WARNING':
        return 'border-yellow-500 bg-yellow-50';
      case 'ERROR':
        return 'border-red-500 bg-red-50';
      case 'SYSTEM':
        return 'border-blue-500 bg-blue-50';
      case 'ROBOT':
        return 'border-green-500 bg-green-50';
      case 'EMERGENCY':
        return 'border-red-600 bg-red-50';
      default:
        return 'border-gray-500 bg-gray-50';
    }
  };

  // 타임스탬프 포맷팅
  const formatTimestamp = (timestamp) => {
    return new Date(timestamp).toLocaleString('ko-KR', {
      hour: '2-digit',
      minute: '2-digit',
      month: 'short',
      day: 'numeric',
    });
  };

  // 에러 상태 처리
  if (error) {
    return (
      <div className="bg-white p-4 rounded-lg shadow">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">최근 이벤트</h2>
        <div className="bg-red-50 border border-red-200 text-red-700 px-4 py-8 rounded flex items-center justify-center">
          <AlertTriangle className="mr-2" size={20} />
          <p>{error}</p>
        </div>
      </div>
    );
  }

  // 데이터가 없는 경우 처리
  if (!events || events.length === 0) {
    return (
      <div className="bg-white p-4 rounded-lg shadow">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">최근 이벤트</h2>
        <div className="text-gray-500 p-4 text-center">
          이벤트 데이터가 없습니다
        </div>
      </div>
    );
  }

  return (
    <div className="bg-white p-4 rounded-lg shadow">
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-lg font-semibold text-gray-800">최근 이벤트</h2>
        <div className="relative">
          <button
            className="flex items-center text-sm text-gray-600 hover:text-gray-800"
            onClick={() => setFilterMenuOpen(!filterMenuOpen)}
          >
            필터 <ChevronDown size={16} className="ml-1" />
          </button>
          {filterMenuOpen && (
            <div className="absolute right-0 mt-2 bg-white border border-gray-200 rounded shadow-lg z-10 p-2 w-48">
              <div className="py-1">
                <label className="flex items-center px-3 py-2 hover:bg-gray-100 cursor-pointer">
                  <input
                    type="checkbox"
                    checked={filters.WARNING}
                    onChange={() => toggleFilter('WARNING')}
                    className="mr-2"
                  />
                  <AlertTriangle size={16} className="text-yellow-500 mr-2" />
                  경고
                </label>
                <label className="flex items-center px-3 py-2 hover:bg-gray-100 cursor-pointer">
                  <input
                    type="checkbox"
                    checked={filters.ERROR}
                    onChange={() => toggleFilter('ERROR')}
                    className="mr-2"
                  />
                  <AlertCircle size={16} className="text-red-500 mr-2" />
                  오류
                </label>
                <label className="flex items-center px-3 py-2 hover:bg-gray-100 cursor-pointer">
                  <input
                    type="checkbox"
                    checked={filters.SYSTEM}
                    onChange={() => toggleFilter('SYSTEM')}
                    className="mr-2"
                  />
                  <Server size={16} className="text-blue-500 mr-2" />
                  시스템
                </label>
                <label className="flex items-center px-3 py-2 hover:bg-gray-100 cursor-pointer">
                  <input
                    type="checkbox"
                    checked={filters.ROBOT}
                    onChange={() => toggleFilter('ROBOT')}
                    className="mr-2"
                  />
                  <Clock size={16} className="text-green-500 mr-2" />
                  로봇
                </label>
                <label className="flex items-center px-3 py-2 hover:bg-gray-100 cursor-pointer">
                  <input
                    type="checkbox"
                    checked={filters.EMERGENCY}
                    onChange={() => toggleFilter('EMERGENCY')}
                    className="mr-2"
                  />
                  <AlertOctagon size={16} className="text-red-600 mr-2" />
                  긴급
                </label>
              </div>
            </div>
          )}
        </div>
      </div>

      <div className="space-y-3">
        {filteredEvents && filteredEvents.length > 0 ? (
          filteredEvents.map((event) => (
            <div
              key={event.id}
              className={`p-3 border-l-4 ${getEventColor(event.type)} rounded cursor-pointer hover:shadow-md transition-shadow`}
              onClick={() => onSelectEvent && onSelectEvent(event)}
            >
              <div className="flex items-start">
                <div className="mr-3 mt-1">{getEventIcon(event.type)}</div>
                <div className="flex-1">
                  <p className="font-medium">{event.message}</p>
                  <div className="flex justify-between text-xs text-gray-500 mt-1">
                    <span>{event.location || '위치 정보 없음'}</span>
                    <span>{formatTimestamp(event.timestamp)}</span>
                  </div>
                </div>
              </div>
            </div>
          ))
        ) : (
          <div className="text-gray-500 p-4 text-center">
            표시할 이벤트가 없습니다. 필터 설정을 확인해주세요.
          </div>
        )}
      </div>
    </div>
  );
};

export default EventTimeline; 