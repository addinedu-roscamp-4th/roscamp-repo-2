import React, { useState } from 'react';
import { Filter, Clock, AlertTriangle, AlertOctagon, AlertCircle, Server, Info, CheckCircle } from 'react-feather';

// EventTimeline 컴포넌트 - 최근 시스템 로그 타임라인 표시
const EventTimeline = ({ logs = [], onSelectLog, error, isLoading, className = ''}) => {
  const [showFilterMenu, setShowFilterMenu] = useState(false);
  const [activeFilters, setActiveFilters] = useState(['ALL']);

  // 로그 필터링 함수
  const filterLogs = () => {
    if (!logs || logs.length === 0) return [];
    
    if (activeFilters.includes('ALL')) {
      return logs;
    }
    
    return logs.filter(log => 
      activeFilters.some(filter => log.level?.toUpperCase().includes(filter))
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

  // 로그 아이콘 가져오기
  const getLogIcon = (logLevel) => {
    switch (logLevel?.toUpperCase()) {
      case 'INFO':
        return <Info className="text-blue-500" size={18} />;
      case 'WARNING':
        return <AlertTriangle className="text-yellow-500" size={18} />;
      case 'ERROR':
        return <AlertOctagon className="text-red-500" size={18} />;
      case 'DEBUG':
        return <CheckCircle className="text-green-500" size={18} />;
      default:
        return <AlertCircle className="text-gray-500" size={18} />;
    }
  };

  // 로그 색상 가져오기
  const getLogColor = (logLevel) => {
    switch (logLevel?.toUpperCase()) {
      case 'INFO':
        return 'border-blue-500';
      case 'WARNING':
        return 'border-yellow-500';
      case 'ERROR':
        return 'border-red-500';
      case 'DEBUG':
        return 'border-green-500';
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

  // 필터링된 로그
  const filteredLogs = filterLogs();

  return (
    <div className={`bg-white … p-4 flex flex-col ${className}`}>
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-lg font-semibold text-gray-700 flex items-center">
          <Clock className="mr-2 text-blue-500" />
          시스템 로그
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
                  모든 로그
                </button>
                <button 
                  className={`w-full text-left px-4 py-2 text-sm ${
                    activeFilters.includes('INFO') ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
                  }`}
                  onClick={() => toggleFilter('INFO')}
                >
                  정보 (INFO)
                </button>
                <button 
                  className={`w-full text-left px-4 py-2 text-sm ${
                    activeFilters.includes('WARNING') ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
                  }`}
                  onClick={() => toggleFilter('WARNING')}
                >
                  경고 (WARNING)
                </button>
                <button 
                  className={`w-full text-left px-4 py-2 text-sm ${
                    activeFilters.includes('ERROR') ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
                  }`}
                  onClick={() => toggleFilter('ERROR')}
                >
                  오류 (ERROR)
                </button>
                <button 
                  className={`w-full text-left px-4 py-2 text-sm ${
                    activeFilters.includes('DEBUG') ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
                  }`}
                  onClick={() => toggleFilter('DEBUG')}
                >
                  디버그 (DEBUG)
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
      ) : filteredLogs.length === 0 ? (
        <div className="text-center py-10">
          <Clock className="mx-auto h-10 w-10 text-gray-400" />
          <h3 className="mt-2 text-sm font-medium text-gray-900">로그 없음</h3>
          <p className="mt-1 text-sm text-gray-500">
            {activeFilters.includes('ALL') 
              ? '아직 기록된 시스템 로그가 없습니다.' 
              : '선택한 필터에 해당하는 로그가 없습니다.'}
          </p>
        </div>
      ) : (
        // 로그 목록
        <div className="space-y-4 overflow-y-auto pr-2 flex-1">
          {filteredLogs.map((log) => (
            <div 
              key={log.id} 
              className={`border-l-4 ${getLogColor(log.level)} pl-3 py-2 hover:bg-gray-50 rounded cursor-pointer`}
              onClick={() => onSelectLog && onSelectLog(log)}
            >
              <div className="flex justify-between items-start">
                <div className="flex items-start">
                  <span className="mr-2 mt-0.5">
                    {getLogIcon(log.level)}
                  </span>
                  <div>
                    <p className="text-sm font-medium text-gray-900">{log.message}</p>
                    <p className="text-xs text-gray-500">{log.level}</p>
                  </div>
                </div>
                <span className="text-xs text-gray-500 whitespace-nowrap ml-2">
                  {formatTimestamp(log.timestamp)}
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