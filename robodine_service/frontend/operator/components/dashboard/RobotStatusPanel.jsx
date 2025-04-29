import React, { useState } from 'react';
import { Battery, Server, Activity } from 'react-feather';
import RobotDetailModal from './RobotDetailModal';

const RobotStatusPanel = ({ robots }) => {
  const [selectedRobot, setSelectedRobot] = useState(null);
  const [isModalOpen, setIsModalOpen] = useState(false);

  const openRobotDetails = (robot) => {
    setSelectedRobot(robot);
    setIsModalOpen(true);
  };

  const closeRobotDetails = () => {
    setIsModalOpen(false);
  };

  // Function to simulate battery level (in real app, this would come from the robot data)
  const getBatteryLevel = (robotId) => {
    const idNumber = parseInt(robotId, 10) || 1;
    return Math.max(20, Math.min(100, (idNumber * 17) % 100));
  };

  const getRobotIcon = (type) => {
    switch (type) {
      case 'COOKBOT':
        return '🍳';
      case 'ALBABOT':
        return '🍽️';
      case 'PINKY':
        return '🤖';
      default:
        return '🤖';
    }
  };

  const getStatusColor = (status) => {
    switch (status) {
      case 'IDLE':
        return 'bg-blue-100 text-blue-800';
      case 'COOKING':
        return 'bg-yellow-100 text-yellow-800';
      case 'SERVING':
        return 'bg-green-100 text-green-800';
      case 'CLEANING':
        return 'bg-indigo-100 text-indigo-800';
      case 'EMERGENCY':
        return 'bg-red-100 text-red-800';
      case 'MAINTENANCE':
        return 'bg-purple-100 text-purple-800';
      case 'SECURITY':
        return 'bg-gray-100 text-gray-800';
      default:
        return 'bg-gray-100 text-gray-800';
    }
  };

  if (!robots || robots.length === 0) {
    return (
      <div className="bg-white rounded-lg shadow p-4 flex flex-col h-full">
        <h2 className="text-lg font-semibold mb-4">로봇 현황</h2>
        <div className="flex items-center justify-center h-full">
          <p className="text-gray-500">등록된 로봇이 없습니다</p>
        </div>
      </div>
    );
  }

  return (
    <div className="bg-white rounded-lg shadow p-4">
      <h2 className="text-lg font-semibold mb-4">로봇 현황</h2>
      <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
        {robots.map((robot) => {
          const batteryLevel = getBatteryLevel(robot.robot_id);
          const batteryColor = 
            batteryLevel > 70 ? 'text-green-500' : 
            batteryLevel > 30 ? 'text-yellow-500' : 
            'text-red-500';

          return (
            <div key={robot.robot_id} className="bg-gray-50 rounded-lg p-4 flex flex-col shadow-sm">
              <div className="flex items-center mb-3">
                <span className="text-xl mr-2">{getRobotIcon(robot.type)}</span>
                <div>
                  <h3 className="font-medium">{robot.type}</h3>
                  <p className="text-xs text-gray-500">ID: {robot.robot_id}</p>
                </div>
              </div>
              
              <div className="flex justify-between items-center mb-3">
                <span className={`px-2 py-1 rounded-full text-xs font-medium ${getStatusColor(robot.status)}`}>
                  {robot.status || 'UNKNOWN'}
                </span>
                
                <div className="flex items-center">
                  <Battery className={batteryColor} size={16} />
                  <span className={`text-xs ml-1 ${batteryColor}`}>{batteryLevel}%</span>
                </div>
              </div>
              
              <div className="flex items-center text-xs text-gray-500 mb-2">
                <Server size={14} className="mr-1" />
                <span>{robot.ip_address || '192.168.1.X'}</span>
              </div>
              
              <div className="flex items-center text-xs text-gray-500 mb-3">
                <Activity size={14} className="mr-1" />
                <span>최근 활동: {new Date(robot.timestamp).toLocaleString()}</span>
              </div>
              
              <button
                onClick={() => openRobotDetails(robot)}
                className="mt-auto text-blue-600 text-sm hover:text-blue-800 focus:outline-none"
              >
                상세 정보 보기
              </button>
            </div>
          );
        })}
      </div>

      {isModalOpen && selectedRobot && (
        <RobotDetailModal
          robot={selectedRobot}
          onClose={closeRobotDetails}
          batteryLevel={getBatteryLevel(selectedRobot.robot_id)}
        />
      )}
    </div>
  );
};

export default RobotStatusPanel;