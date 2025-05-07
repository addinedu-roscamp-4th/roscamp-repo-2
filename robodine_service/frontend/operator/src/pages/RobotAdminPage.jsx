import React, { useState, useEffect, useCallback, useMemo } from 'react';
import { 
  Edit, Trash2, Send, AlertTriangle, 
  Clock, Terminal, Briefcase, Power,
  PlusCircle, X, Save, RefreshCw,
  Activity, ToggleLeft
} from 'lucide-react';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useWebSockets } from '../contexts/WebSocketContext';

// 웹소켓 토픽 리스트 (Context와 동일하게 유지)
const TOPICS = ['robots', 'status'];

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

// 로봇 상태 목록 (backend의 RobotStatus enum과 일치)
const ROBOT_STATUSES = [
  { value: 'IDLE', label: '대기 중', color: 'bg-green-100 text-green-800' },
  { value: 'COOKING', label: '조리 중', color: 'bg-yellow-100 text-yellow-800' },
  { value: 'SERVING', label: '서빙 중', color: 'bg-blue-100 text-blue-800' },
  { value: 'CLEANING', label: '청소 중', color: 'bg-indigo-100 text-indigo-800' },
  { value: 'EMERGENCY', label: '비상', color: 'bg-red-100 text-red-800' },
  { value: 'SECURITY', label: '보안 모드', color: 'bg-purple-100 text-purple-800' },
  { value: 'CHARGING', label: '충전 중', color: 'bg-blue-100 text-blue-800' },
  { value: 'ERROR', label: '오류', color: 'bg-red-100 text-red-800' },
  { value: 'MAINTENANCE', label: '정비 중', color: 'bg-orange-100 text-orange-800' }
];

const getStatusColor = (status) => {
  const statusObj = ROBOT_STATUSES.find(s => s.value === status);
  return statusObj ? statusObj.color : 'bg-gray-100 text-gray-800';
};

const getStatusLabel = (status) => {
  const statusObj = ROBOT_STATUSES.find(s => s.value === status);
  return statusObj ? statusObj.label : status;
};

const RobotTable = ({ robots, onEditRobot, onDeleteRobot, onSendCommand, onChangeStatus }) => {
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
                <div className="flex items-center">
                  <span className={`px-2 py-1 text-xs rounded-full ${getStatusColor(robot.status)}`}>
                    {getStatusLabel(robot.status)}
                  </span>
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      onChangeStatus(robot);
                    }}
                    className="ml-2 flex items-center justify-center px-2 py-1 bg-blue-50 text-blue-700 rounded-md hover:bg-blue-100"
                    title="상태 변경"
                  >
                    <Activity size={14} className="mr-1" />
                    <span className="text-xs">상태 변경</span>
                  </button>
                </div>
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
              <td className="px-6 py-4 whitespace-nowrap">
                <div className="flex space-x-4">
                  <button 
                    onClick={() => onEditRobot(robot)}
                    className="flex items-center justify-center px-2 py-1 bg-blue-50 text-blue-700 rounded-md hover:bg-blue-100"
                    title="로봇 정보 수정"
                  >
                    <Edit size={16} className="mr-1" />
                    <span className="text-xs">수정</span>
                  </button>
                  <button 
                    onClick={() => onDeleteRobot(robot.id)}
                    className="flex items-center justify-center px-2 py-1 bg-red-50 text-red-700 rounded-md hover:bg-red-100"
                    title="로봇 삭제"
                  >
                    <Trash2 size={16} className="mr-1" />
                    <span className="text-xs">삭제</span>
                  </button>
                  <button 
                    onClick={() => onSendCommand(robot)}
                    className="flex items-center justify-center px-2 py-1 bg-purple-50 text-purple-700 rounded-md hover:bg-purple-100"
                    title="명령 전송"
                  >
                    <Send size={16} className="mr-1" />
                    <span className="text-xs">명령</span>
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
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [isAddRobotModalOpen, setIsAddRobotModalOpen] = useState(false);
  const [isEditRobotModalOpen, setIsEditRobotModalOpen] = useState(false);
  const [isCommandModalOpen, setIsCommandModalOpen] = useState(false);
  const [isStatusModalOpen, setIsStatusModalOpen] = useState(false);
  const [currentRobot, setCurrentRobot] = useState(null);
  const [currentCommand, setCurrentCommand] = useState({
    command: '',
    parameters: ''
  });
  const [selectedStatus, setSelectedStatus] = useState('');
  const { apiCall } = useAuth();
  const { data, errors, connected, refreshTopic } = useWebSockets();
  const { robots, commands, status } = data;
  const [lastUpdateTime, setLastUpdateTime] = useState(new Date());
  const [apiActionStatus, setApiActionStatus] = useState({ type: '', message: '', isError: false });

  // 로봇 데이터 처리
  const processedRobots = useMemo(() => {
    // status.robots 배열이 있으면 이걸 쓰고, 없으면 fallback
    const base = Array.isArray(status.robots)
      ? status.robots
      : Array.isArray(robots)
        ? robots
        : [];
  
    return base.map(r => {
      // 1) 기본 필드
      const robot = {
        id: r['Robot.id'] || r.robot_id,
        robot_id: r['Robot.robot_id'] || r.robot_id,
        type: r['Robot.type'] || r.robot_type || 'UNKNOWN',
        ipAddress: r['Robot.ip_address'] || r.ip_address,
        macAddress: r['Robot.mac_address'] || r.mac_address,
        lastActive: r['Robot.timestamp'] || r.timestamp,
      };
  
      // 2) albabot/cookbot 상태
      const alb = (status.albabots || []).find(a => a['Albabot.robot_id'] === robot.robot_id);
      if (alb) {
        robot.status = alb['Albabot.status'] || 'OFFLINE';
        const lvl = alb['Albabot.battery_level'];
        robot.battery = lvl <= 1 ? Math.round(lvl * 100) : Math.round(lvl);
      } else {
        const cook = (status.cookbots || []).find(c => c['Cookbot.robot_id'] === robot.robot_id);
        if (cook) {
          robot.status = cook['Cookbot.status'] || 'OFFLINE';
          robot.battery = 100;
        } else {
          robot.status = 'OFFLINE';
          robot.battery = 0;
        }
      }
  
      robot.name = `Robot #${robot.id}`;
      return robot;
    });
  }, [status, robots]);

  // 명령 데이터 처리
  const processedCommands = useMemo(() => {
    const cmdList = Array.isArray(commands) ? commands : [];
    return cmdList.map(c => ({
      id: c['Command.id'] || c.id,
      robot_id: c['Command.robot_id'] || c.robot_id,
      command: c['Command.command'] || c.command,
      parameters: c['Command.parameters'] || c.parameters,
      status: c['Command.status'] || c.status,
      issued_at: c['Command.issued_at'] || c.issued_at
    }));
  }, [commands]);

  // 수동 새로고침 핸들러
  const handleRefreshData = useCallback(() => {
    setIsLoading(true);
    TOPICS.forEach(topic => refreshTopic(topic));
    setTimeout(() => setIsLoading(false), 2000);
  }, [refreshTopic]);

  // API 응답 처리 및 알림 표시
  const handleApiResponse = useCallback((type, isSuccess, message) => {
    setApiActionStatus({
      type,
      message: isSuccess 
        ? `${message} 완료되었습니다.` 
        : `${message} 실패했습니다.`,
      isError: !isSuccess
    });
    
    // 3초 후 알림 자동 제거
    setTimeout(() => {
      setApiActionStatus({ type: '', message: '', isError: false });
    }, 3000);
  }, []);

  // 업데이트 시 로딩 해제 및 타임스탬프 갱신
  useEffect(() => {
    setLastUpdateTime(new Date());
    setIsLoading(false);
  }, [robots, commands, status]);

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
      setIsLoading(true);
      await apiCall(`/api/robots/${id}`, 'DELETE');
      handleApiResponse('delete', true, '로봇 삭제가');
      refreshTopic('robots');
    } catch (error) {
      console.error(`Failed to delete robot ${id}:`, error);
      handleApiResponse('delete', false, '로봇 삭제가');
      setError('로봇 삭제 중 오류가 발생했습니다');
    } finally {
      setIsLoading(false);
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
    if (!currentCommand.command) {
      setError('명령을 선택해주세요');
      return;
    }
    
    try {
      setIsLoading(true);
      await apiCall(`/api/robots/commands/${currentRobot.id}/command`, 'POST', {
        robot_id: currentRobot.id,
        command: currentCommand.command,
        parameter: JSON.parse(currentCommand.parameters || '{}')
      });
      
      handleApiResponse('command', true, '명령 전송이');
      refreshTopic('commands');
      setIsCommandModalOpen(false);
    } catch (error) {
      console.error('Failed to send command:', error);
      handleApiResponse('command', false, '명령 전송이');
      setError('명령 전송 중 오류가 발생했습니다');
    } finally {
      setIsLoading(false);
    }
  };

  // 로봇 추가/수정 저장
  const handleSaveRobot = async () => {
    // 입력값 검증
    if (!currentRobot.id || !currentRobot.type || !currentRobot.ipAddress) {
      setError('필수 정보를 모두 입력해주세요');
      return;
    }
    
    try {
      setIsLoading(true);
      if (isAddRobotModalOpen) {
        // 새 로봇 등록
        const response = await apiCall('/api/robots/register', 'POST', {
          robot_id: parseInt(currentRobot.id),
          robot_type: currentRobot.type,
          mac_address: currentRobot.macAddress || '',
          ip_address: currentRobot.ipAddress
        });
        
        if (response.status === 'success') {
          handleApiResponse('add', true, '로봇 등록이');
          refreshTopic('robots');
          setIsAddRobotModalOpen(false);
        }
      } else if (isEditRobotModalOpen) {
        // 기존 로봇 정보 수정
        await apiCall(`/api/robots/${currentRobot.id}`, 'PUT', {
          robot_id: parseInt(currentRobot.id),
          robot_type: currentRobot.type,
          mac_address: currentRobot.macAddress || '',
          ip_address: currentRobot.ipAddress
        });
        
        handleApiResponse('edit', true, '로봇 정보 수정이');
        refreshTopic('robots');
        setIsEditRobotModalOpen(false);
      }
    } catch (error) {
      console.error('Failed to save robot:', error);
      handleApiResponse('save', false, '로봇 정보 저장이');
      setError('로봇 정보 저장 중 오류가 발생했습니다');
    } finally {
      setIsLoading(false);
    }
  };

  // 로봇 상태 변경 모달 열기
  const handleChangeStatus = (robot) => {
    setCurrentRobot(robot);
    setSelectedStatus(robot.status || 'IDLE');
    setIsStatusModalOpen(true);
  };

  // 로봇 상태 변경 처리
  const handleSubmitStatusChange = async () => {
    if (!selectedStatus) {
      setError('상태를 선택해주세요');
      return;
    }

    try {
      setIsLoading(true);
      
      // 알바봇인지 쿡봇인지 확인하여 적절한 API 호출
      if (currentRobot.type === 'ALBABOT') {
        console.log('알바봇 상태 변경 요청:', {
          robot_id: parseInt(currentRobot.id),
          status: selectedStatus,
          battery_level: currentRobot.battery / 100,
          timestamp: new Date().toISOString()
        });
        
        await apiCall('/api/albabot/status', 'POST', {
          robot_id: parseInt(currentRobot.id),
          status: selectedStatus,
          battery_level: currentRobot.battery,
          timestamp: new Date().toISOString()
        });
      } else if (currentRobot.type === 'COOKBOT') {
        console.log('쿡봇 상태 변경 요청:', {
          robot_id: parseInt(currentRobot.id),
          status: selectedStatus,
          timestamp: new Date().toISOString()
        });
        
        await apiCall('/api/cookbot/status', 'POST', {
          robot_id: parseInt(currentRobot.id),
          status: selectedStatus,
          timestamp: new Date().toISOString()
        });
      }
      
      handleApiResponse('status', true, '로봇 상태 변경이');
      refreshTopic('status');
      setTimeout(() => refreshTopic('robots'), 500);
      setIsStatusModalOpen(false);
    } catch (error) {
      console.error('Failed to change robot status:', error);
      handleApiResponse('status', false, '로봇 상태 변경이');
      setError('로봇 상태 변경 중 오류가 발생했습니다. 올바른 API 경로인지 확인하세요.');
    } finally {
      setIsLoading(false);
    }
  };

  const formatDateTime = date =>
    date.toLocaleString('ko-KR', {
      year: 'numeric', month: 'long', day: 'numeric',
      weekday: 'long', hour: '2-digit', minute: '2-digit'
    });

  return (
    <Layout>
      <div className="container mx-auto px-4 py-6">
        <div className="mb-6 flex flex-col md:flex-row md:items-center md:justify-between">
          <h1 className="text-2xl font-bold text-gray-900 mb-4 md:mb-0">로봇 관리 시스템</h1>
          <div className="flex space-x-2 items-center">
            <p className="text-sm text-gray-500 mr-2">마지막 업데이트: {formatDateTime(lastUpdateTime)}</p>
            {!connected.robots && <span className="text-xs bg-red-100 text-red-800 px-2 py-1 rounded-full mr-2">연결 끊김</span>}
            <button
              onClick={handleRefreshData}
              className="px-3 py-2 bg-white border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 hover:bg-gray-50"
              disabled={isLoading}
            >
              <RefreshCw size={16} className={`inline mr-1 ${isLoading ? 'animate-spin' : ''}`} />
              새로고침
            </button>
            <button
              onClick={handleAddRobot}
              className="px-3 py-2 bg-blue-600 text-white rounded-md shadow-sm text-sm font-medium hover:bg-blue-700"
              disabled={isLoading}
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
        
        {apiActionStatus.message && (
          <div className={`${apiActionStatus.isError ? 'bg-red-100 border-red-500 text-red-700' : 'bg-green-100 border-green-500 text-green-700'} border-l-4 p-4 mb-4 rounded transition-opacity duration-500`}>
            <p>{apiActionStatus.message}</p>
          </div>
        )}
        
        {errors.robots && (
          <div className="bg-yellow-100 border-l-4 border-yellow-500 text-yellow-700 p-4 mb-4 rounded">
            <div className="flex">
              <AlertTriangle className="h-5 w-5 mr-2" />
              <p>로봇 데이터 가져오기 오류: {errors.robots}</p>
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
                  robots={processedRobots} 
                  onEditRobot={handleEditRobot} 
                  onDeleteRobot={handleDeleteRobot} 
                  onSendCommand={handleSendCommand}
                  onChangeStatus={handleChangeStatus}
                />
              </>
            ) : (
              <CommandList commands={processedCommands} />
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
                로봇 ID <span className="text-red-500">*</span>
              </label>
              <input
                type="text"
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentRobot.id}
                onChange={(e) => setCurrentRobot({...currentRobot, id: e.target.value})}
                placeholder="숫자 ID"
              />
            </div>
            
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                로봇 타입 <span className="text-red-500">*</span>
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
                IP 주소 <span className="text-red-500">*</span>
              </label>
              <input
                type="text"
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentRobot.ipAddress}
                onChange={(e) => setCurrentRobot({...currentRobot, ipAddress: e.target.value})}
                placeholder="예: 192.168.1.100"
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
                placeholder="예: 00:1A:2B:3C:4D:5E"
              />
            </div>
            
            <div className="flex justify-end">
              <button
                onClick={() => setIsAddRobotModalOpen(false)}
                className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
                disabled={isLoading}
              >
                취소
              </button>
              <button
                onClick={handleSaveRobot}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 flex items-center"
                disabled={isLoading}
              >
                {isLoading ? (
                  <RefreshCw size={16} className="animate-spin mr-2" />
                ) : (
                  <Save size={16} className="mr-2" />
                )}
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
                로봇 타입 <span className="text-red-500">*</span>
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
                IP 주소 <span className="text-red-500">*</span>
              </label>
              <input
                type="text"
                className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                value={currentRobot.ipAddress}
                onChange={(e) => setCurrentRobot({...currentRobot, ipAddress: e.target.value})}
                placeholder="예: 192.168.1.100"
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
                placeholder="예: 00:1A:2B:3C:4D:5E"
              />
            </div>
            
            <div className="flex justify-end">
              <button
                onClick={() => setIsEditRobotModalOpen(false)}
                className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
                disabled={isLoading}
              >
                취소
              </button>
              <button
                onClick={handleSaveRobot}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 flex items-center"
                disabled={isLoading}
              >
                {isLoading ? (
                  <RefreshCw size={16} className="animate-spin mr-2" />
                ) : (
                  <Save size={16} className="mr-2" />
                )}
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
                명령어 <span className="text-red-500">*</span>
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
              <div className="mt-2 text-xs text-gray-600">
                <p className="font-semibold mb-1">명령어 파라미터 예시:</p>
                <ul className="space-y-1 list-disc pl-5">
                  <li><span className="font-medium">MOVE</span>: {"{'x': 100, 'y': 200}"}</li>
                  <li><span className="font-medium">PICKUP</span>: {"{'item_id': 123}"}</li>
                  <li><span className="font-medium">CHARGE</span>: {"{'duration': 30}"} (분)</li>
                </ul>
              </div>
            </div>
            
            <div className="flex justify-end">
              <button
                onClick={() => setIsCommandModalOpen(false)}
                className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
                disabled={isLoading}
              >
                취소
              </button>
              <button
                onClick={handleSubmitCommand}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 flex items-center"
                disabled={!currentCommand.command || isLoading}
              >
                {isLoading ? (
                  <RefreshCw size={16} className="animate-spin mr-2" />
                ) : (
                  <Send size={16} className="mr-2" />
                )}
                전송
              </button>
            </div>
          </div>
        </div>
      )}

      {/* 로봇 상태 변경 모달 */}
      {isStatusModalOpen && currentRobot && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl p-6 max-w-md w-full mx-4">
            <div className="flex justify-between items-center mb-4">
              <h3 className="text-lg font-medium text-gray-900">
                로봇 #{currentRobot.id} 상태 변경
              </h3>
              <button
                onClick={() => setIsStatusModalOpen(false)}
                className="text-gray-400 hover:text-gray-500"
              >
                <X size={20} />
              </button>
            </div>
            
            <div className="mb-6">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                현재 상태:
              </label>
              <span className={`inline-block px-2 py-1 text-sm rounded-full ${getStatusColor(currentRobot.status)}`}>
                {getStatusLabel(currentRobot.status)}
              </span>
            </div>
            
            <div className="mb-6">
              <label className="block text-sm font-medium text-gray-700 mb-2">
                새 상태: <span className="text-red-500">*</span>
              </label>
              <div className="grid grid-cols-2 gap-2">
                {ROBOT_STATUSES.map(status => (
                  <button
                    key={status.value}
                    onClick={() => setSelectedStatus(status.value)}
                    className={`px-3 py-2 border rounded-md text-sm ${
                      selectedStatus === status.value
                        ? 'border-blue-500 bg-blue-50 text-blue-800'
                        : 'border-gray-300 hover:bg-gray-50'
                    }`}
                  >
                    <span className={`inline-block w-3 h-3 rounded-full mr-2 ${status.color.split(' ')[0]}`}></span>
                    {status.label}
                  </button>
                ))}
              </div>
            </div>
            
            <div className="flex justify-end">
              <button
                onClick={() => setIsStatusModalOpen(false)}
                className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
                disabled={isLoading}
              >
                취소
              </button>
              <button
                onClick={handleSubmitStatusChange}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 flex items-center"
                disabled={!selectedStatus || isLoading}
              >
                {isLoading ? (
                  <RefreshCw size={16} className="animate-spin mr-2" />
                ) : (
                  <Activity size={16} className="mr-2" />
                )}
                적용
              </button>
            </div>
          </div>
        </div>
      )}

    </Layout>
  );
};

export default RobotAdminPage;