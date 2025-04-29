import React, { useState } from 'react';
import { Server, Battery, Clock, AlertTriangle, PlusCircle, CheckCircle } from 'react-feather';

const RobotStatusPanel = ({ robots = [], error, isLoading }) => {
  const [expandedRobot, setExpandedRobot] = useState(null);

  // 로봇 상태에 따른 색상
  const getStatusColor = (status) => {
    switch (status?.toUpperCase()) {
      case 'ONLINE':
        return 'bg-green-100 text-green-800';
      case 'OFFLINE':
        return 'bg-gray-100 text-gray-800';
      case 'BUSY':
        return 'bg-yellow-100 text-yellow-800';
      case 'ERROR':
        return 'bg-red-100 text-red-800';
      default:
        return 'bg-gray-100 text-gray-800';
    }
  };

  // 배터리 레벨에 따른 색상
  const getBatteryColor = (level) => {
    if (level < 20) return 'bg-red-500';
    if (level < 50) return 'bg-yellow-500';
    return 'bg-green-500';
  };

  // 마지막 활동 시간 포맷팅
  const formatLastActive = (timestamp) => {
    if (!timestamp) return '알 수 없음';
    
    const now = new Date();
    const lastActive = new Date(timestamp);
    const diffMs = now - lastActive;
    const diffMins = Math.floor(diffMs / 60000);
    
    if (diffMins < 1) return '방금 전';
    if (diffMins < 60) return `${diffMins}분 전`;
    
    const diffHours = Math.floor(diffMins / 60);
    if (diffHours < 24) return `${diffHours}시간 전`;
    
    return lastActive.toLocaleDateString('ko-KR');
  };

  // 로봇 카드 클릭 처리
  const handleRobotClick = (id) => {
    if (expandedRobot === id) {
      setExpandedRobot(null);
    } else {
      setExpandedRobot(id);
    }
  };

  // 로봇 IP 주소 포맷팅
  const formatIpAddress = (ip) => {
    return ip || '192.168.1.x';
  };

  return (
    <div className="bg-white rounded-lg shadow p-4 h-full">
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-lg font-semibold text-gray-700 flex items-center">
          <Server className="mr-2 text-blue-500" />
          로봇 상태
        </h2>
        <span className="text-sm text-gray-500">
          {robots?.length || 0}대 로봇
        </span>
      </div>

      {isLoading ? (
        <div className="flex items-center justify-center h-64">
          <div className="animate-spin rounded-full h-8 w-8 border-t-2 border-b-2 border-blue-500"></div>
        </div>
      ) : error ? (
        <div className="bg-red-50 text-red-600 p-4 rounded-md flex items-center">
          <AlertTriangle className="mr-2" size={20} />
          <span>{error}</span>
        </div>
      ) : robots?.length === 0 ? (
        <div className="text-center py-10">
          <Server className="mx-auto h-10 w-10 text-gray-400" />
          <h3 className="mt-2 text-sm font-medium text-gray-900">로봇 없음</h3>
          <p className="mt-1 text-sm text-gray-500">등록된 로봇이 없습니다.</p>
        </div>
      ) : (
        <div className="space-y-3 max-h-72 overflow-y-auto pr-2">
          {robots.map((robot) => (
            <div 
              key={robot.id} 
              className="border rounded-lg hover:shadow-md transition-shadow cursor-pointer overflow-hidden"
              onClick={() => handleRobotClick(robot.id)}
            >
              <div className="p-3">
                <div className="flex justify-between items-start">
                  <div className="flex items-start">
                    <div className={`p-2 rounded-md mr-3 ${
                      robot.status?.toUpperCase() === 'ONLINE' ? 'bg-green-100' : 
                      robot.status?.toUpperCase() === 'BUSY' ? 'bg-yellow-100' : 
                      'bg-gray-100'
                    }`}>
                      <Server size={20} className={
                        robot.status?.toUpperCase() === 'ONLINE' ? 'text-green-600' : 
                        robot.status?.toUpperCase() === 'BUSY' ? 'text-yellow-600' : 
                        'text-gray-600'
                      } />
                    </div>
                    <div>
                      <div className="flex items-center">
                        <h3 className="font-medium text-gray-900">{robot.name || `로봇 #${robot.id}`}</h3>
                        <span className={`ml-2 px-2 py-0.5 text-xs rounded-full ${getStatusColor(robot.status)}`}>
                          {robot.status || 'UNKNOWN'}
                        </span>
                      </div>
                      <p className="text-sm text-gray-500">{robot.type || '운송 로봇'}</p>
                    </div>
                  </div>
                  <div className="flex items-center">
                    <Battery size={16} className={
                      (robot.battery || 0) < 20 ? 'text-red-500' : 
                      (robot.battery || 0) < 50 ? 'text-yellow-500' : 
                      'text-green-500'
                    } />
                    <span className="text-xs ml-1 text-gray-600">{robot.battery || 0}%</span>
                  </div>
                </div>
                
                {expandedRobot === robot.id && (
                  <div className="mt-3 pt-3 border-t text-sm">
                    <div className="grid grid-cols-2 gap-2">
                      <div className="text-gray-500">IP 주소:</div>
                      <div className="text-gray-800 font-mono">{formatIpAddress(robot.ipAddress)}</div>
                      
                      <div className="text-gray-500">마지막 활동:</div>
                      <div className="text-gray-800 flex items-center">
                        <Clock size={14} className="mr-1" />
                        {formatLastActive(robot.lastActive)}
                      </div>
                      
                      {robot.position && (
                        <>
                          <div className="text-gray-500">위치:</div>
                          <div className="text-gray-800 font-mono">
                            X: {robot.position.x || 0}, Y: {robot.position.y || 0}
                          </div>
                        </>
                      )}
                      
                      <div className="text-gray-500">상태:</div>
                      <div className="text-gray-800 flex items-center">
                        {robot.status?.toUpperCase() === 'ONLINE' ? 
                          <CheckCircle size={14} className="mr-1 text-green-500" /> : 
                          <PlusCircle size={14} className="mr-1 text-gray-500" />
                        }
                        {robot.status?.toUpperCase() === 'ONLINE' ? '정상 작동 중' : '상태 확인 필요'}
                      </div>
                    </div>
                  </div>
                )}
              </div>
              
              {/* 배터리 프로그레스 바 */}
              <div className="w-full bg-gray-200 h-1">
                <div 
                  className={`h-1 ${getBatteryColor(robot.battery || 0)}`}
                  style={{ width: `${robot.battery || 0}%` }}
                ></div>
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default RobotStatusPanel;