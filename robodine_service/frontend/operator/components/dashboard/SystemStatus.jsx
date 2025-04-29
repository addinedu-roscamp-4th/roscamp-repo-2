import React from 'react';
import { AlertTriangle, Server, Check, XCircle, AlertOctagon } from 'react-feather';

const SystemStatus = ({ systems, error }) => {
  // 에러 상태 처리
  if (error) {
    return (
      <div className="bg-white p-4 rounded-lg shadow">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">시스템 상태</h2>
        <div className="bg-red-50 border border-red-200 text-red-700 px-4 py-8 rounded flex items-center justify-center">
          <AlertTriangle className="mr-2" size={20} />
          <p>{error}</p>
        </div>
      </div>
    );
  }

  // 데이터가 없는 경우 처리
  if (!systems || systems.length === 0) {
    return (
      <div className="bg-white p-4 rounded-lg shadow">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">시스템 상태</h2>
        <div className="text-gray-500 p-4 text-center">
          시스템 상태 데이터가 없습니다
        </div>
      </div>
    );
  }

  // 상태에 따른 아이콘과 색상 가져오기
  const getStatusIcon = (status) => {
    switch (status) {
      case 'ONLINE':
        return <Check size={18} className="text-green-500" />;
      case 'OFFLINE':
        return <XCircle size={18} className="text-red-500" />;
      case 'WARNING':
        return <AlertTriangle size={18} className="text-yellow-500" />;
      case 'ERROR':
        return <AlertOctagon size={18} className="text-red-600" />;
      default:
        return <Server size={18} className="text-gray-500" />;
    }
  };

  // 상태에 따른 배지 색상 가져오기
  const getStatusBadgeClass = (status) => {
    switch (status) {
      case 'ONLINE':
        return 'bg-green-100 text-green-800';
      case 'OFFLINE':
        return 'bg-red-100 text-red-800';
      case 'WARNING':
        return 'bg-yellow-100 text-yellow-800';
      case 'ERROR':
        return 'bg-red-100 text-red-800';
      default:
        return 'bg-gray-100 text-gray-800';
    }
  };

  return (
    <div className="bg-white p-4 rounded-lg shadow">
      <h2 className="text-lg font-semibold text-gray-800 mb-4">시스템 상태</h2>
      <div className="divide-y">
        {systems.map((system) => (
          <div key={system.id} className="py-3">
            <div className="flex justify-between items-center">
              <div className="flex items-center">
                <div className="mr-3">{getStatusIcon(system.status)}</div>
                <div>
                  <p className="font-medium">{system.name}</p>
                  <p className="text-sm text-gray-500">{system.description}</p>
                </div>
              </div>
              <div className={`px-2 py-1 rounded-full text-xs font-medium ${getStatusBadgeClass(system.status)}`}>
                {system.status}
              </div>
            </div>
            <p className="text-xs text-gray-500 mt-1">
              마지막 업데이트: {new Date(system.lastUpdated).toLocaleString('ko-KR', {
                hour: '2-digit',
                minute: '2-digit',
                month: 'short',
                day: 'numeric'
              })}
            </p>
          </div>
        ))}
      </div>
    </div>
  );
};

export default SystemStatus; 