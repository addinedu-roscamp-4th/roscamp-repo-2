import React, { useState, useEffect } from 'react';
import { X, Send, RotateCw } from 'react-feather';

const RobotDetailModal = ({ robot, onClose, batteryLevel }) => {
  const [commands, setCommands] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [commandInput, setCommandInput] = useState('');
  const [paramInput, setParamInput] = useState('{}');
  const [error, setError] = useState(null);

  useEffect(() => {
    // Fetch robot commands
    const fetchCommands = async () => {
      setIsLoading(true);
      try {
        const response = await fetch(`/api/robot/commands/${robot.robot_id}`);
        if (!response.ok) throw new Error('Failed to fetch robot commands');
        const data = await response.json();
        setCommands(data);
      } catch (error) {
        console.error('Error fetching robot commands:', error);
        setError('명령 내역을 불러오는 중 오류가 발생했습니다');
      } finally {
        setIsLoading(false);
      }
    };

    fetchCommands();
  }, [robot.robot_id]);

  const handleSendCommand = async (e) => {
    e.preventDefault();
    setError(null);

    if (!commandInput.trim()) {
      setError('명령을 입력해주세요');
      return;
    }

    // Validate JSON parameter
    let parameters;
    try {
      parameters = JSON.parse(paramInput);
    } catch (error) {
      setError('유효한 JSON 형식이 아닙니다');
      return;
    }

    setIsLoading(true);

    try {
      const response = await fetch(`/api/robot/commands/${robot.robot_id}/command`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          robot_id: parseInt(robot.robot_id, 10),
          command: commandInput,
          parameter: parameters,
        }),
      });

      if (!response.ok) throw new Error('Failed to send command');
      const result = await response.json();

      // Refresh commands
      const updatedCommandsResponse = await fetch(`/api/robot/commands/${robot.robot_id}`);
      if (updatedCommandsResponse.ok) {
        const updatedCommands = await updatedCommandsResponse.json();
        setCommands(updatedCommands);
      }

      // Clear inputs
      setCommandInput('');
      setParamInput('{}');
    } catch (error) {
      console.error('Error sending command:', error);
      setError('명령 전송 중 오류가 발생했습니다');
    } finally {
      setIsLoading(false);
    }
  };

  const formatTimestamp = (timestamp) => {
    const date = new Date(timestamp);
    return date.toLocaleString();
  };

  const getStatusColor = (status) => {
    switch (status) {
      case 'PENDING':
        return 'bg-yellow-100 text-yellow-800';
      case 'SENT':
        return 'bg-blue-100 text-blue-800';
      case 'ACKED':
        return 'bg-purple-100 text-purple-800';
      case 'EXECUTED':
        return 'bg-green-100 text-green-800';
      case 'FAILED':
        return 'bg-red-100 text-red-800';
      default:
        return 'bg-gray-100 text-gray-800';
    }
  };

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div className="bg-white rounded-lg shadow-lg w-full max-w-2xl max-h-[80vh] overflow-hidden flex flex-col">
        <div className="flex justify-between items-center p-4 border-b">
          <h2 className="text-lg font-semibold">
            {robot.type} (ID: {robot.robot_id})
          </h2>
          <button
            onClick={onClose}
            className="text-gray-500 hover:text-gray-700 focus:outline-none"
          >
            <X size={20} />
          </button>
        </div>

        <div className="p-6 overflow-y-auto flex-1">
          <div className="grid grid-cols-2 gap-4 mb-6">
            <div>
              <h3 className="text-sm font-medium text-gray-500 mb-1">MAC 주소</h3>
              <p className="text-gray-800">{robot.mac_address || 'N/A'}</p>
            </div>
            <div>
              <h3 className="text-sm font-medium text-gray-500 mb-1">IP 주소</h3>
              <p className="text-gray-800">{robot.ip_address || 'N/A'}</p>
            </div>
            <div>
              <h3 className="text-sm font-medium text-gray-500 mb-1">상태</h3>
              <span className={`px-2 py-1 rounded-full text-xs font-medium ${getStatusColor(robot.status)}`}>
                {robot.status || 'UNKNOWN'}
              </span>
            </div>
            <div>
              <h3 className="text-sm font-medium text-gray-500 mb-1">배터리</h3>
              <div className="w-full bg-gray-200 rounded-full h-2.5">
                <div
                  className={`h-2.5 rounded-full ${
                    batteryLevel > 70 ? 'bg-green-500' : batteryLevel > 30 ? 'bg-yellow-400' : 'bg-red-500'
                  }`}
                  style={{ width: `${batteryLevel}%` }}
                ></div>
              </div>
              <p className="text-xs mt-1">{batteryLevel}%</p>
            </div>
            <div>
              <h3 className="text-sm font-medium text-gray-500 mb-1">최근 업데이트</h3>
              <p className="text-gray-800">{formatTimestamp(robot.timestamp)}</p>
            </div>
          </div>

          <div className="mb-6">
            <h3 className="text-md font-semibold mb-3">명령 전송</h3>
            {error && (
              <div className="mb-4 bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded text-sm">
                {error}
              </div>
            )}
            <form onSubmit={handleSendCommand} className="space-y-3">
              <div>
                <label htmlFor="command" className="block text-sm font-medium text-gray-700 mb-1">
                  명령
                </label>
                <input
                  type="text"
                  id="command"
                  value={commandInput}
                  onChange={(e) => setCommandInput(e.target.value)}
                  placeholder="예: move_to, cook, clean"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                />
              </div>
              <div>
                <label htmlFor="parameters" className="block text-sm font-medium text-gray-700 mb-1">
                  파라미터 (JSON)
                </label>
                <textarea
                  id="parameters"
                  value={paramInput}
                  onChange={(e) => setParamInput(e.target.value)}
                  placeholder='{"key": "value"}'
                  rows="3"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                />
              </div>
              <div>
                <button
                  type="submit"
                  disabled={isLoading}
                  className="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md shadow-sm text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 disabled:opacity-50"
                >
                  {isLoading ? '전송 중...' : '명령 전송'}
                  <Send size={16} className="ml-2" />
                </button>
              </div>
            </form>
          </div>

          <div>
            <div className="flex justify-between items-center mb-3">
              <h3 className="text-md font-semibold">최근 명령 내역</h3>
              <button
                onClick={() => {
                  const fetchCommands = async () => {
                    setIsLoading(true);
                    try {
                      const response = await fetch(`/api/robot/commands/${robot.robot_id}`);
                      if (!response.ok) throw new Error('Failed to fetch robot commands');
                      const data = await response.json();
                      setCommands(data);
                    } catch (error) {
                      console.error('Error fetching robot commands:', error);
                      setError('명령 내역을 불러오는 중 오류가 발생했습니다');
                    } finally {
                      setIsLoading(false);
                    }
                  };
                  fetchCommands();
                }}
                className="text-blue-600 hover:text-blue-800 text-sm flex items-center"
              >
                <RotateCw size={14} className="mr-1" />
                새로고침
              </button>
            </div>
            
            {isLoading ? (
              <div className="flex items-center justify-center h-32">
                <div className="animate-spin rounded-full h-6 w-6 border-t-2 border-b-2 border-blue-500"></div>
              </div>
            ) : commands.length > 0 ? (
              <div className="space-y-2 max-h-60 overflow-y-auto">
                {commands.map((cmd) => (
                  <div key={cmd.id} className="border rounded-lg p-3">
                    <div className="flex justify-between items-start">
                      <div className="font-medium">{cmd.command}</div>
                      <span className={`px-2 py-1 rounded-full text-xs font-medium ${getStatusColor(cmd.status)}`}>
                        {cmd.status}
                      </span>
                    </div>
                    <div className="text-xs text-gray-500 mt-1">
                      {cmd.issued_at ? formatTimestamp(cmd.issued_at) : 'Unknown time'}
                    </div>
                  </div>
                ))}
              </div>
            ) : (
              <div className="text-center py-8 text-gray-500">명령 내역이 없습니다</div>
            )}
          </div>
        </div>

        <div className="bg-gray-50 px-4 py-3 sm:px-6 sm:flex sm:flex-row-reverse border-t">
          <button
            type="button"
            onClick={onClose}
            className="w-full inline-flex justify-center rounded-md border border-gray-300 shadow-sm px-4 py-2 bg-white text-base font-medium text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 sm:mt-0 sm:ml-3 sm:w-auto sm:text-sm"
          >
            닫기
          </button>
        </div>
      </div>
    </div>
  );
};

export default RobotDetailModal; 