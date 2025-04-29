import React from 'react';
import { Battery, AlertTriangle } from 'react-feather';

const RobotStatusPanel = ({ robots, error }) => {
  // 에러 상태 처리
  if (error) {
    return (
      <div className="bg-white p-4 rounded-lg shadow">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">로봇 상태</h2>
        <div className="bg-red-50 border border-red-200 text-red-700 px-4 py-8 rounded flex items-center justify-center">
          <AlertTriangle className="mr-2" size={20} />
          <p>{error}</p>
        </div>
      </div>
    );
  }

  // 데이터가 없는 경우 처리
  if (!robots || robots.length === 0) {
    return (
      <div className="bg-white p-4 rounded-lg shadow">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">로봇 상태</h2>
        <div className="text-gray-500 p-4 text-center">
          로봇 데이터가 없습니다
        </div>
      </div>
    );
  }

  // 로봇 상태에 따른 색상 설정
  const getStatusColor = (status) => {
    switch (status) {
      case 'ONLINE':
        return 'bg-green-500';
      case 'OFFLINE':
        return 'bg-gray-500';
      case 'BUSY':
        return 'bg-yellow-500';
      case 'ERROR':
        return 'bg-red-500';
      default:
        return 'bg-gray-500';
    }
  };

  // 배터리 상태에 따른 색상 설정
  const getBatteryColor = (level) => {
    if (level < 20) return 'text-red-500';
    if (level < 50) return 'text-yellow-500';
    return 'text-green-500';
  };

  return (
    <div className="bg-white p-4 rounded-lg shadow">
      <h2 className="text-lg font-semibold text-gray-800 mb-4">로봇 상태</h2>
      <div className="overflow-x-auto">
        <table className="min-w-full">
          <thead>
            <tr className="bg-gray-100">
              <th className="px-4 py-2 text-left">로봇</th>
              <th className="px-4 py-2 text-left">상태</th>
              <th className="px-4 py-2 text-left">배터리</th>
              <th className="px-4 py-2 text-left">마지막 활동</th>
            </tr>
          </thead>
          <tbody>
            {robots.map((robot) => (
              <tr key={robot.id} className="border-b">
                <td className="px-4 py-2">
                  <div className="font-medium">{robot.name}</div>
                  <div className="text-xs text-gray-500">{robot.type}</div>
                </td>
                <td className="px-4 py-2">
                  <div className="flex items-center">
                    <div className={`w-3 h-3 rounded-full ${getStatusColor(robot.status)} mr-2`}></div>
                    {robot.status}
                  </div>
                </td>
                <td className="px-4 py-2">
                  <div className="flex items-center">
                    <Battery className={`mr-1 ${getBatteryColor(robot.battery)}`} size={16} />
                    <span className={getBatteryColor(robot.battery)}>{robot.battery}%</span>
                  </div>
                </td>
                <td className="px-4 py-2">
                  {new Date(robot.lastActive).toLocaleString('ko-KR', {
                    hour: '2-digit',
                    minute: '2-digit',
                    month: 'short',
                    day: 'numeric'
                  })}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default RobotStatusPanel; 