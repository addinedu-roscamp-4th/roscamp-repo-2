import React, { useState, useEffect, useCallback } from 'react';
import { 
  Edit, Trash2, Play, Pause, Send, AlertTriangle, 
  Clock, Terminal, Briefcase, ToggleRight, Power,
  PlusCircle, X, Save, RefreshCw
} from 'lucide-react';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';

// 웹소켓 설정
const WS_BASE_URL = 'ws://127.0.0.1:8000/ws';

// 웹소켓 커스텀 훅
const useWebSocket = (topic, onMessageReceived) => {
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  
  useEffect(() => {
    const ws = new WebSocket(`${WS_BASE_URL}/${topic}`);
    
    ws.onopen = () => {
      console.log(`${topic} 웹소켓 연결됨`);
      setConnected(true);
      setError(null);
    };
    
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.type === 'update' && data.topic === topic) {
          onMessageReceived(data.data);
        }
      } catch (error) {
        console.error(`${topic} 웹소켓 메시지 처리 오류:`, error);
      }
    };
    
    ws.onclose = () => {
      console.log(`${topic} 웹소켓 연결 종료`);
      setConnected(false);
    };
    
    ws.onerror = (error) => {
      console.error(`${topic} 웹소켓 오류:`, error);
      setError(`연결 오류: ${error.message || '알 수 없는 오류'}`);
    };
    
    return () => {
      ws.close();
    };
  }, [topic, onMessageReceived]);
  
  return { connected, error };
};

const RobotTable = ({ robots, onEditRobot, onDeleteRobot, onToggleStatus, onSendCommand }) => {
  if (!robots || robots.length === 0) {
    return (
      <div className="text-center py-10 bg-gray-50 rounded-lg">
        <Power className="mx-auto h-12 w-12 text-gray-400" />
        <h3 className="mt-2 text-sm font-medium text-gray-900">로봇 없음</h3>
        <p className="mt-1 text-sm text-gray-500">시스템에 등록된 로봇이 없습니다.</p>
      </div>
    );
  }

  return (
    <div className="overflow-x-auto">
      <table className="min-w-full divide-y divide-gray-200">
        <thead className="bg-gray-50">
          <tr>
            <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
              ID
            </th>
            <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
              로봇 정보
            </th>
            <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
              IP 주소
            </th>
            <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
              상태
            </th>
            <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
              배터리
            </th>
            <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
              작업
            </th>
          </tr>
        </thead>
        <tbody className="bg-white divide-y divide-gray-200">
          {robots.map((robot) => (
            <tr key={robot.id} className="hover:bg-gray-50">
              <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">
                #{robot.id}
              </td>
              <td className="px-6 py-4 whitespace-nowrap">
                <div className="flex items-center">
                  <div>
                    <div className="text-sm font-medium text-gray-900">{robot.name}</div>
                    <div className="text-sm text-gray-500">{robot.type}</div>
                  </div>
                </div>
              </td>
              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                {robot.ipAddress}
              </td>
              <td className="px-6 py-4 whitespace-nowrap">
                <span className={`px-2 py-1 text-xs rounded-full ${
                  robot.status === 'ONLINE' ? 'bg-green-100 text-green-800' : 
                  robot.status === 'BUSY' ? 'bg-yellow-100 text-yellow-800' : 
                  'bg-gray-100 text-gray-800'
                }`}>
                  {robot.status}
                </span>
              </td>
              <td className="px-6 py-4 whitespace-nowrap">
                <div className="w-16 bg-gray-200 rounded-full h-2.5">
                  <div 
                    className={`h-2.5 rounded-full ${
                      robot.battery < 20 ? 'bg-red-500' : 
                      robot.battery < 50 ? 'bg-yellow-500' : 
                      'bg-green-500'
                    }`}
                    style={{ width: `${robot.battery}%` }}
                  ></div>
                </div>
                <span className="text-xs text-gray-500 ml-1">{robot.battery}%</span>
              </td>
              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                <div className="flex space-x-2">
                  <button 
                    onClick={() => onEditRobot(robot)}
                    className="text-blue-600 hover:text-blue-900"
                    title="로봇 정보 수정"
                  >
                    <Edit size={18} />
                  </button>
                  <button 
                    onClick={() => onDeleteRobot(robot.id)}
                    className="text-red-600 hover:text-red-900"
                    title="로봇 삭제"
                  >
                    <Trash2 size={18} />
                  </button>
                  <button 
                    onClick={() => onToggleStatus(robot.id, robot.status === 'ONLINE' ? 'OFFLINE' : 'ONLINE')}
                    className={`${robot.status === 'ONLINE' ? 'text-green-600 hover:text-green-900' : 'text-gray-600 hover:text-gray-900'}`}
                    title={robot.status === 'ONLINE' ? '로봇 중지' : '로봇 시작'}
                  >
                    {robot.status === 'ONLINE' ? <Pause size={18} /> : <Play size={18} />}
                  </button>
                  <button 
                    onClick={() => onSendCommand(robot)}
                    className="text-purple-600 hover:text-purple-900"
                    title="명령 전송"
                  >
                    <Send size={18} />
                  </button>
                </div>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

const CommandList = ({ commands }) => {
  if (!commands || commands.length === 0) {
    return (
      <div className="text-center py-10 bg-gray-50 rounded-lg">
        <Terminal className="mx-auto h-12 w-12 text-gray-400" />
        <h3 className="mt-2 text-sm font-medium text-gray-900">명령어 없음</h3>
        <p className="mt-1 text-sm text-gray-500">최근 실행된 명령이 없습니다.</p>
      </div>
    );
  }

  const getStatusBadge = (status) => {
    switch (status) {
      case 'SUCCESS':
        return <span className="px-2 py-1 text-xs rounded-full bg-green-100 text-green-800">성공</span>;
      case 'FAILED':
        return <span className="px-2 py-1 text-xs rounded-full bg-red-100 text-red-800">실패</span>;
      case 'PENDING':
        return <span className="px-2 py-1 text-xs rounded-full bg-yellow-100 text-yellow-800">대기중</span>;
      default:
        return <span className="px-2 py-1 text-xs rounded-full bg-gray-100 text-gray-800">{status}</span>;
    }
  };

  return (
    <div className="bg-white shadow overflow-hidden rounded-md">
      <ul className="divide-y divide-gray-200">
        {commands.map((command) => (
          <li key={command.id} className="p-4 hover:bg-gray-50">
            <div className="flex justify-between">
              <div className="flex items-center">
                <div className="bg-gray-100 rounded-lg p-2 mr-3">
                  <Send size={16} className="text-gray-500" />
                </div>
                <div>
                  <p className="text-sm font-medium text-gray-900">{command.command}</p>
                  <p className="text-xs text-gray-500">{command.parameters}</p>
                </div>
              </div>
              <div className="flex items-center">
                {getStatusBadge(command.status)}
                <span className="text-xs text-gray-500 ml-2 flex items-center">
                  <Clock size={12} className="mr-1" />
                  {new Date(command.issued_at).toLocaleString('ko-KR')}
                </span>
              </div>
            </div>
          </li>
        ))}
      </ul>
    </div>
  );
};

const RobotAdminPage = () => {
  const [activeTab, setActiveTab] = useState('robots');
  const [robots, setRobots] = useState([]);
  const [commands, setCommands] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [isAddRobotModalOpen, setIsAddRobotModalOpen] = useState(false);
  const [isEditRobotModalOpen, setIsEditRobotModalOpen] = useState(false);
  const [isCommandModalOpen, setIsCommandModalOpen] = useState(false);
  const [currentRobot, setCurrentRobot] = useState(null);
  const [currentCommand, setCurrentCommand] = useState({
    command: '',
    parameters: ''
  });
  const { apiCall } = useAuth();

  // 로봇 데이터 가져오기
  const fetchRobots = useCallback(async () => {
    setIsLoading(true);
    setError(null);
    try {
      const robotsData = await apiCall('/api/robots');
      
      // 서버 응답 형식에 맞게 데이터 변환
      const formattedRobots = robotsData.map(robot => ({
        id: robot.robot_id,
        name: `Robot #${robot.robot_id}`,
        type: robot.robot_type,
        ipAddress: robot.ip_address,
        macAddress: robot.mac_address,
        status: robot.status || 'OFFLINE',
        battery: robot.battery || Math.floor(Math.random() * 100), // 배터리 정보가 없으면 랜덤 값
        timestamp: robot.timestamp
      }));
      
      setRobots(formattedRobots);
      
      // 로봇 명령어 목록 가져오기
      try {
        const commandsData = await apiCall('/api/robots/commands');
        setCommands(commandsData);
      } catch (cmdError) {
        console.error('Failed to load robot commands:', cmdError);
        // 실패 시 모의 데이터 사용
        setCommands([
          {
            id: 1,
            robot_id: 1,
            command: 'MOVE',
            parameters: 'x=150, y=200',
            status: 'SUCCESS',
            issued_at: new Date(Date.now() - 3600000).toISOString()
          },
          {
            id: 2,
            robot_id: 2,
            command: 'PICKUP',
            parameters: 'item_id=123',
            status: 'FAILED',
            issued_at: new Date(Date.now() - 7200000).toISOString()
          },
          {
            id: 3,
            robot_id: 1,
            command: 'CHARGE',
            parameters: '',
            status: 'PENDING',
            issued_at: new Date(Date.now() - 1800000).toISOString()
          }
        ]);
      }
    } catch (err) {
      console.error('Failed to load robots:', err);
      setError('로봇 정보를 불러올 수 없습니다');
    } finally {
      setIsLoading(false);
    }
  }, [apiCall]);

  // 초기 데이터 로딩
  useEffect(() => {
    fetchRobots();
  }, [fetchRobots]);

  // 웹소켓 업데이트 처리
  const handleRobotsUpdate = useCallback((data) => {
    if (Array.isArray(data)) {
      const updatedRobots = data.map(robot => ({
        id: robot['Robot.id'] || robot.id,
        name: `Robot #${robot['Robot.id'] || robot.id}`,
        type: robot['Robot.type'] || robot.type || 'UNKNOWN',
        ipAddress: robot['Robot.ip_address'] || robot.ip_address,
        macAddress: robot['Robot.mac_address'] || robot.mac_address,
        status: robot['Robot.status'] || robot.status || 'OFFLINE',
        battery: robot['Robot.battery'] || robot.battery || Math.floor(Math.random() * 100),
        timestamp: robot['Robot.timestamp'] || robot.timestamp
      }));
      
      setRobots(prevRobots => {
        // 기존 로봇 데이터와 병합
        const robotMap = new Map();
        
        // 기존 로봇을 맵에 추가
        prevRobots.forEach(robot => {
          robotMap.set(robot.id, robot);
        });
        
        // 업데이트된 로봇으로 맵 갱신
        updatedRobots.forEach(robot => {
          robotMap.set(robot.id, robot);
        });
        
        // 맵을 배열로 변환하여 반환
        return Array.from(robotMap.values());
      });
    }
  }, []);

  // 웹소켓 연결
  const robotsWS = useWebSocket('robots', handleRobotsUpdate);

  // 로봇 편집 모달 열기
  const handleEditRobot = (robot) => {
    setCurrentRobot(robot);
    setIsEditRobotModalOpen(true);
  };

  // 로봇 추가 모달 열기
  const handleAddRobot = () => {
    setCurrentRobot({
      id: '',
      name: '',
      type: 'ALBABOT',
      ipAddress: '',
      macAddress: '',
      status: 'OFFLINE',
      battery: 100
    });
    setIsAddRobotModalOpen(true);
  };

  // 로봇 삭제 처리
  const handleDeleteRobot = async (id) => {
    if (!window.confirm(`로봇 #${id}을 삭제하시겠습니까?`)) {
      return;
    }
    
    try {
      await apiCall(`/api/robots/${id}`, 'DELETE');
      setRobots(robots.filter(robot => robot.id !== id));
    } catch (error) {
      console.error(`Failed to delete robot ${id}:`, error);
      setError('로봇 삭제 중 오류가 발생했습니다');
    }
  };

  // 로봇 상태 토글
  const handleToggleStatus = async (id, newStatus) => {
    try {
      await apiCall(`/api/robots/${id}/status`, 'PUT', { status: newStatus });
      
      // 로봇 목록 업데이트
      setRobots(robots.map(robot => 
        robot.id === id ? { ...robot, status: newStatus } : robot
      ));
    } catch (error) {
      console.error(`Failed to update robot ${id} status:`, error);
      setError('로봇 상태 변경 중 오류가 발생했습니다');
    }
  };

  // 로봇 명령 전송 모달 열기
  const handleSendCommand = (robot) => {
    setCurrentRobot(robot);
    setCurrentCommand({
      command: '',
      parameters: ''
    });
    setIsCommandModalOpen(true);
  };

  // 로봇 명령 전송 처리
  const handleSubmitCommand = async () => {
    try {
      await apiCall(`/api/robots/commands/${currentRobot.id}/command`, 'POST', {
        robot_id: currentRobot.id,
        command: currentCommand.command,
        parameter: JSON.parse(currentCommand.parameters || '{}')
      });
      
      // 명령 목록에 새 명령 추가
      const newCommand = {
        id: Date.now(), // 임시 ID
        robot_id: currentRobot.id,
        command: currentCommand.command,
        parameters: currentCommand.parameters,
        status: 'PENDING',
        issued_at: new Date().toISOString()
      };
      
      setCommands([newCommand, ...commands]);
      setIsCommandModalOpen(false);
    } catch (error) {
      console.error('Failed to send command:', error);
      setError('명령 전송 중 오류가 발생했습니다');
    }
  };

  // 로봇 추가/수정 저장
  const handleSaveRobot = async () => {
    try {
      if (isAddRobotModalOpen) {
        // 새 로봇 등록
        const response = await apiCall('/api/robots/register', 'POST', {
          robot_id: parseInt(currentRobot.id),
          robot_type: currentRobot.type,
          mac_address: currentRobot.macAddress,
          ip_address: currentRobot.ipAddress
        });
        
        if (response.status === 'success') {
          // 로봇 목록에 새 로봇 추가
          setRobots([...robots, currentRobot]);
          setIsAddRobotModalOpen(false);
        }
      } else if (isEditRobotModalOpen) {
        // 기존 로봇 정보 수정
        await apiCall(`/api/robots/${currentRobot.id}`, 'PUT', {
          robot_type: currentRobot.type,
          mac_address: currentRobot.macAddress,
          ip_address: currentRobot.ipAddress
        });
        
        // 로봇 목록 업데이트
        setRobots(robots.map(robot => 
          robot.id === currentRobot.id ? currentRobot : robot
        ));
        setIsEditRobotModalOpen(false);
      }
    } catch (error) {
      console.error('Failed to save robot:', error);
      setError('로봇 정보 저장 중 오류가 발생했습니다');
    }
  };

  return (
    <Layout>
      <div className="container mx-auto px-4 py-6">
        <div className="mb-6 flex flex-col md:flex-row md:items-center md:justify-between">
          <h1 className="text-2xl font-bold text-gray-900 mb-4 md:mb-0">로봇 관리 시스템</h1>
          <div className="flex space-x-2">
            <button
              onClick={fetchRobots}
              className="px-3 py-2 bg-white border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 hover:bg-gray-50"
            >
              <RefreshCw size={16} className="inline mr-1" />
              새로고침
            </button>
            <button
              onClick={handleAddRobot}
              className="px-3 py-2 bg-blue-600 text-white rounded-md shadow-sm text-sm font-medium hover:bg-blue-700"
            >
              <PlusCircle size={16} className="inline mr-1" />
              로봇 추가
            </button>
          </div>
        </div>
        
        {error && (
          <div className="bg-red-100 border-l-4 border-red-500 text-red-700 p-4 mb-4 rounded">
            <div className="flex">
              <AlertTriangle className="h-5 w-5 mr-2" />
              <p>{error}</p>
            </div>
          </div>
        )}
        
        <div className="bg-white shadow rounded-lg overflow-hidden">
          <div className="flex border-b border-gray-200">
            <button
              className={`px-4 py-3 text-sm font-medium ${
                activeTab === 'robots'
                  ? 'text-blue-600 border-b-2 border-blue-500'
                  : 'text-gray-500 hover:text-gray-700'
              }`}
              onClick={() => setActiveTab('robots')}
            >
              <Briefcase size={16} className="inline mr-1" />
              로봇 목록
            </button>
            <button
              className={`px-4 py-3 text-sm font-medium ${
                activeTab === 'commands'
                  ? 'text-blue-600 border-b-2 border-blue-500'
                  : 'text-gray-500 hover:text-gray-700'
              }`}
              onClick={() => setActiveTab('commands')}
            >
              <Terminal size={16} className="inline mr-1" />
              명령어 로그
            </button>
          </div>

          <div className="p-4">
            {isLoading ? (
              <div className="flex justify-center items-center h-48">
                <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
              </div>
            ) : activeTab === 'robots' ? (
              <>
                <RobotTable 
                  robots={robots} 
                  onEditRobot={handleEditRobot} 
                  onDeleteRobot={handleDeleteRobot} 
                  onToggleStatus={handleToggleStatus}
                  onSendCommand={handleSendCommand}
                />
              </>
            ) : (
              <CommandList commands={commands} />
            )}
          </div>
        </div>
      </div>

      {/* 로봇 추가 모달 */}
      {isAddRobotModalOpen && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl p-6 max-w-md w-full mx-4">
            <div className="flex justify-between items-center mb-4">
              <h3 className="text-lg font-medium text-gray-900">
                새 로봇 추가
              </h3>
              <button
                onClick={() => setIsAddRobotModalOpen(false)}
                className="text-gray-400 hover:text-gray-500"
              >
                <X size={20} />
              </button>
            </div>
            
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                로봇 ID
              </label>
              <input
                type="text"
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentRobot.id}
                onChange={(e) => setCurrentRobot({...currentRobot, id: e.target.value})}
              />
            </div>
            
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                로봇 타입
              </label>
              <select
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentRobot.type}
                onChange={(e) => setCurrentRobot({...currentRobot, type: e.target.value})}
              >
                <option value="ALBABOT">알바봇</option>
                <option value="COOKBOT">쿡봇</option>
              </select>
            </div>
            
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                IP 주소
              </label>
              <input
                type="text"
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentRobot.ipAddress}
                onChange={(e) => setCurrentRobot({...currentRobot, ipAddress: e.target.value})}
              />
            </div>
            
            <div className="mb-6">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                MAC 주소
              </label>
              <input
                type="text"
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentRobot.macAddress}
                onChange={(e) => setCurrentRobot({...currentRobot, macAddress: e.target.value})}
              />
            </div>
            
            <div className="flex justify-end">
              <button
                onClick={() => setIsAddRobotModalOpen(false)}
                className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
              >
                취소
              </button>
              <button
                onClick={handleSaveRobot}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 flex items-center"
              >
                <Save size={16} className="mr-2" />
                저장
              </button>
            </div>
          </div>
        </div>
      )}

      {/* 로봇 편집 모달 */}
      {isEditRobotModalOpen && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl p-6 max-w-md w-full mx-4">
            <div className="flex justify-between items-center mb-4">
              <h3 className="text-lg font-medium text-gray-900">
                로봇 #{currentRobot.id} 정보 수정
              </h3>
              <button
                onClick={() => setIsEditRobotModalOpen(false)}
                className="text-gray-400 hover:text-gray-500"
              >
                <X size={20} />
              </button>
            </div>
            
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                로봇 타입
              </label>
              <select
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentRobot.type}
                onChange={(e) => setCurrentRobot({...currentRobot, type: e.target.value})}
              >
                <option value="ALBABOT">알바봇</option>
                <option value="COOKBOT">쿡봇</option>
              </select>
            </div>
            
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                IP 주소
              </label>
              <input
                type="text"
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentRobot.ipAddress}
                onChange={(e) => setCurrentRobot({...currentRobot, ipAddress: e.target.value})}
              />
            </div>
            
            <div className="mb-6">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                MAC 주소
              </label>
              <input
                type="text"
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentRobot.macAddress}
                onChange={(e) => setCurrentRobot({...currentRobot, macAddress: e.target.value})}
              />
            </div>
            
            <div className="flex justify-end">
              <button
                onClick={() => setIsEditRobotModalOpen(false)}
                className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
              >
                취소
              </button>
              <button
                onClick={handleSaveRobot}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 flex items-center"
              >
                <Save size={16} className="mr-2" />
                저장
              </button>
            </div>
          </div>
        </div>
      )}

      {/* 명령 전송 모달 */}
      {isCommandModalOpen && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl p-6 max-w-md w-full mx-4">
            <div className="flex justify-between items-center mb-4">
              <h3 className="text-lg font-medium text-gray-900">
                로봇 #{currentRobot.id}에 명령 전송
              </h3>
              <button
                onClick={() => setIsCommandModalOpen(false)}
                className="text-gray-400 hover:text-gray-500"
              >
                <X size={20} />
              </button>
            </div>
            
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                명령어
              </label>
              <select
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentCommand.command}
                onChange={(e) => setCurrentCommand({...currentCommand, command: e.target.value})}
              >
                <option value="">명령 선택</option>
                <option value="MOVE">이동</option>
                <option value="PICKUP">물품 집기</option>
                <option value="PUTDOWN">물품 놓기</option>
                <option value="CHARGE">충전</option>
                <option value="STOP">정지</option>
                <option value="RESET">리셋</option>
              </select>
            </div>
            
            <div className="mb-6">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                파라미터 (JSON 형식)
              </label>
              <textarea
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                rows="4"
                placeholder='{"x": 100, "y": 200}'
                value={currentCommand.parameters}
                onChange={(e) => setCurrentCommand({...currentCommand, parameters: e.target.value})}
              ></textarea>
              <p className="mt-1 text-xs text-gray-500">
                JSON 형식으로 파라미터를 입력하세요. 예: {"{'x': 100, 'y': 200}"}
              </p>
            </div>
            
            <div className="flex justify-end">
              <button
                onClick={() => setIsCommandModalOpen(false)}
                className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
              >
                취소
              </button>
              <button
                onClick={handleSubmitCommand}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 flex items-center"
                disabled={!currentCommand.command}
              >
                <Send size={16} className="mr-2" />
                전송
              </button>
            </div>
          </div>
        </div>
      )}
    </Layout>
  );
};

export default RobotAdminPage;