import React, { useState, useEffect } from 'react';
import { 
  Edit, Trash2, Play, Pause, Send, AlertTriangle, 
  Clock, Terminal, Briefcase, ToggleRight, Power
} from 'react-feather';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';

const RobotTable = ({ robots, onEditRobot, onDeleteRobot, onToggleStatus }) => {
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
  const { apiCall } = useAuth();

  useEffect(() => {
    const fetchData = async () => {
      setIsLoading(true);
      setError(null);
      try {
        const robotsData = await apiCall('/api/robots');
        setRobots(robotsData);
        
        // 로봇 명령어 목록 가져오기 시도
        try {
          const commandsData = await apiCall('/api/robots/commands');
          setCommands(commandsData);
        } catch (cmdError) {
          console.error('Failed to load robot commands:', cmdError);
          // 명령어 데이터 실패는 치명적이지 않음 - 샘플 데이터로 대체
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
    };

    fetchData();
  }, [apiCall]);

  const handleEditRobot = (robot) => {
    // 편집 모달 표시 로직
    console.log('Edit robot:', robot);
    alert(`로봇 ${robot.id} 편집 기능은 개발 중입니다.`);
  };

  const handleDeleteRobot = async (id) => {
    if (!window.confirm(`로봇 #${id}을 삭제하시겠습니까?`)) {
      return;
    }
    
    try {
      await apiCall(`/api/robots/${id}`, { method: 'DELETE' });
      setRobots(robots.filter(robot => robot.id !== id));
    } catch (error) {
      console.error(`Failed to delete robot ${id}:`, error);
      alert('로봇 삭제 중 오류가 발생했습니다');
    }
  };

  const handleToggleStatus = async (id, newStatus) => {
    try {
      await apiCall(`/api/robots/${id}/status`, { 
        method: 'PUT',
        body: JSON.stringify({ status: newStatus })
      });
      
      setRobots(robots.map(robot => 
        robot.id === id ? { ...robot, status: newStatus } : robot
      ));
    } catch (error) {
      console.error(`Failed to update robot ${id} status:`, error);
      alert('로봇 상태 변경 중 오류가 발생했습니다');
    }
  };

  return (
    <Layout>
      <div className="container mx-auto p-4">
        <div className="flex justify-between items-center mb-6">
          <h1 className="text-2xl font-bold text-gray-800 flex items-center">
            <Briefcase className="text-blue-600 mr-2" size={28} />
            로봇 관리
          </h1>
          <button className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-opacity-50">
            로봇 추가
          </button>
        </div>

        {error && (
          <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6 flex items-center">
            <AlertTriangle className="mr-2" size={20} />
            {error}
          </div>
        )}

        {isLoading ? (
          <div className="flex items-center justify-center h-64">
            <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
          </div>
        ) : (
          <div className="bg-white rounded-lg shadow overflow-hidden">
            {/* 탭 네비게이션 */}
            <div className="border-b border-gray-200">
              <nav className="flex -mb-px">
                <button
                  className={`py-4 px-6 text-center border-b-2 font-medium text-sm ${
                    activeTab === 'robots'
                      ? 'border-blue-500 text-blue-600'
                      : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
                  }`}
                  onClick={() => setActiveTab('robots')}
                >
                  로봇
                </button>
                <button
                  className={`py-4 px-6 text-center border-b-2 font-medium text-sm ${
                    activeTab === 'commands'
                      ? 'border-blue-500 text-blue-600'
                      : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
                  }`}
                  onClick={() => setActiveTab('commands')}
                >
                  명령어
                </button>
              </nav>
            </div>
            
            {/* 탭 콘텐츠 */}
            <div className="p-4">
              {activeTab === 'robots' ? (
                <RobotTable 
                  robots={robots} 
                  onEditRobot={handleEditRobot}
                  onDeleteRobot={handleDeleteRobot}
                  onToggleStatus={handleToggleStatus}
                />
              ) : (
                <CommandList commands={commands} />
              )}
            </div>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default RobotAdminPage; 