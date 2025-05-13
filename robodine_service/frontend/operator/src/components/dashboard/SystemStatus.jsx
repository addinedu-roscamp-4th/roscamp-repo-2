import React from 'react';
import { Monitor, AlertTriangle, CheckCircle, HelpCircle, Activity, Wifi } from 'react-feather';

const SystemStatus = ({ systems = [], error, isLoading }) => {
  // systems가 배열인지 확인
  const systemsArray = Array.isArray(systems) ? systems : [];

  // 시스템 상태에 따른 아이콘과 색상
  const getStatusInfo = (status) => {
    switch (status?.toUpperCase()) {
      case 'ONLINE':
      case 'OK':
      case 'ACTIVE':
        return {
          icon: <CheckCircle size={18} className="text-green-500" />,
          color: 'text-green-500',
          bgColor: 'bg-green-100'
        };
      case 'WARNING':
        return {
          icon: <AlertTriangle size={18} className="text-yellow-500" />,
          color: 'text-yellow-500',
          bgColor: 'bg-yellow-100'
        };
      case 'ERROR':
      case 'CRITICAL':
      case 'OFFLINE':
        return {
          icon: <AlertTriangle size={18} className="text-red-500" />,
          color: 'text-red-500',
          bgColor: 'bg-red-100'
        };
      default:
        return {
          icon: <HelpCircle size={18} className="text-gray-500" />,
          color: 'text-gray-500',
          bgColor: 'bg-gray-100'
        };
    }
  };

  // 타임스탬프 포맷팅
  const formatLastUpdated = (timestamp) => {
    if (!timestamp) return '알 수 없음';
    const date = new Date(timestamp);
    return date.toLocaleString('ko-KR', { 
      month: 'numeric', 
      day: 'numeric', 
      hour: '2-digit', 
      minute: '2-digit'
    });
  };

  // 시스템 타입에 따른 아이콘
  const getSystemTypeIcon = (type) => {
    switch (type?.toUpperCase()) {
      case 'SERVER':
        return <Monitor size={18} />;
      case 'DATABASE':
        return <Monitor size={18} />;
      case 'SENSOR':
        return <Activity size={18} />;
      case 'WEBSOCKET':
        return <Wifi size={18} />;
      default:
        return <Monitor size={18} />;
    }
  };

  // 시스템 데이터가 없거나 유효하지 않을 때의 메시지
  const getErrorMessage = () => {
    if (error) return error;
    if (!systems) return '시스템 상태 데이터를 불러올 수 없습니다';
    if (!Array.isArray(systems)) return '유효하지 않은 시스템 데이터 형식입니다';
    return '시스템 정보를 가져오는 중 오류가 발생했습니다';
  };

  // 웹소켓 연결 상태 확인
  const hasWebSocketSystem = systemsArray.some(system => 
    system?.type?.toUpperCase() === 'WEBSOCKET'
  );

  // 웹소켓 상태 메시지
  const getWebSocketStatusMessage = () => {
    return '웹소켓이 연결되었으나 실시간 데이터 수신 기능은 추후 구현될 예정입니다.';
  };

  return (
    <div className="bg-white rounded-lg shadow p-4 h-full">
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-lg font-semibold text-gray-700 flex items-center">
          <Monitor className="mr-2 text-blue-500" />
          시스템 상태
        </h2>
      </div>

      {isLoading ? (
        <div className="flex items-center justify-center h-64">
          <div className="animate-spin rounded-full h-8 w-8 border-t-2 border-b-2 border-blue-500"></div>
        </div>
      ) : !Array.isArray(systems) || error ? (
        <div className="bg-red-50 text-red-600 p-4 rounded-md flex items-center">
          <AlertTriangle className="mr-2" size={20} />
          <span>{getErrorMessage()}</span>
        </div>
      ) : systemsArray.length === 0 ? (
        <div className="text-center py-10">
          <Monitor className="mx-auto h-10 w-10 text-gray-400" />
          <h3 className="mt-2 text-sm font-medium text-gray-900">시스템 정보 없음</h3>
          <p className="mt-1 text-sm text-gray-500">시스템 상태 정보가 없습니다.</p>
        </div>
      ) : (
        <div className="space-y-3 max-h-72 overflow-y-auto pr-2">
          {!hasWebSocketSystem && (
            <div className="mb-2 p-3 bg-yellow-50 border border-yellow-200 rounded-lg">
              <div className="flex items-center text-yellow-700">
                <Wifi className="mr-2" size={18} />
                <p className="text-sm">웹소켓 연결 정보가 없습니다. 로봇 위치 데이터를 실시간으로 받을 수 없습니다.</p>
              </div>
            </div>
          )}
          
          <div className="mb-2 p-3 bg-blue-50 border border-blue-200 rounded-lg">
            <div className="flex items-center text-blue-700">
              <AlertTriangle className="mr-2" size={18} />
              <p className="text-sm">시스템 상태 API 기능이 구현되지 않았습니다. 정적 데이터만 표시됩니다.</p>
            </div>
          </div>
          
          {systemsArray.map((system) => {
            const { icon, color, bgColor } = getStatusInfo(system.status);
            
            return (
              <div 
                key={system.id} 
                className="border rounded-lg p-3 hover:shadow-sm transition-shadow"
              >
                <div className="flex justify-between items-start">
                  <div className="flex items-start">
                    <div className={`p-2 rounded-md mr-3 ${bgColor}`}>
                      {getSystemTypeIcon(system.type)}
                    </div>
                    <div>
                      <div className="flex items-center">
                        <h3 className="font-medium text-gray-900">{system.name}</h3>
                        <span className={`ml-2 ${color} text-xs`}>{system.status}</span>
                      </div>
                      <p className="text-sm text-gray-500">{system.type || '일반 시스템'}</p>
                    </div>
                  </div>
                  <span className="text-xs text-gray-500">
                    {formatLastUpdated(system.lastUpdated)}
                  </span>
                </div>
                
                {system.description && (
                  <div className="mt-2 text-sm text-gray-600">
                    {system.description}
                  </div>
                )}
                
                {system.type?.toUpperCase() === 'WEBSOCKET' && system.status?.toUpperCase() === 'ONLINE' && (
                  <div className="mt-2 text-xs text-yellow-600 bg-yellow-50 p-2 rounded">
                    {getWebSocketStatusMessage()}
                  </div>
                )}
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
};

export default SystemStatus; 