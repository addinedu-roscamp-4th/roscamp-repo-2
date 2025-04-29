import React, { useState, useEffect } from 'react';
import Layout from '../components/Layout';
import { AlertCircle, ChevronDown, ChevronUp, Download, RefreshCw, Filter } from 'react-feather';

const SystemPage = () => {
  const [expandedPanel, setExpandedPanel] = useState('events'); // 'events', 'logs', 'emergency'
  const [events, setEvents] = useState([]);
  const [logs, setLogs] = useState([]);
  const [emergencies, setEmergencies] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  
  // Filter state
  const [logLevel, setLogLevel] = useState('ALL');
  const [logLimit, setLogLimit] = useState(100);

  useEffect(() => {
    fetchData();
  }, [logLevel, logLimit]);

  const fetchData = async () => {
    setIsLoading(true);
    setError(null);

    try {
      // Fetch events
      if (expandedPanel === 'events') {
        const eventsResponse = await fetch('/api/events');
        if (!eventsResponse.ok) throw new Error('Failed to fetch events');
        const eventsData = await eventsResponse.json();
        setEvents(eventsData);
      }
      
      // Fetch logs with filters
      if (expandedPanel === 'logs') {
        const logsUrl = logLevel === 'ALL' 
          ? `/api/events/system-logs?limit=${logLimit}`
          : `/api/events/system-logs?level=${logLevel}&limit=${logLimit}`;
          
        const logsResponse = await fetch(logsUrl);
        if (!logsResponse.ok) throw new Error('Failed to fetch logs');
        const logsData = await logsResponse.json();
        setLogs(logsData);
      }
      
      // Fetch emergencies (events with EMERGENCY type)
      if (expandedPanel === 'emergency') {
        const eventsResponse = await fetch('/api/events');
        if (!eventsResponse.ok) throw new Error('Failed to fetch events');
        const eventsData = await eventsResponse.json();
        const emergencyEvents = eventsData.filter(event => event.type === 'EMERGENCY');
        setEmergencies(emergencyEvents);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
      setError('데이터를 불러오는 중 오류가 발생했습니다');
    } finally {
      setIsLoading(false);
    }
  };

  const handlePanelToggle = (panel) => {
    if (expandedPanel === panel) {
      // Don't close panels in this UI
      return;
    }
    setExpandedPanel(panel);
    fetchData();
  };

  const formatTimestamp = (timestamp) => {
    return new Date(timestamp).toLocaleString();
  };

  const exportData = (type) => {
    let dataToExport = [];
    let filename = '';
    
    if (type === 'events') {
      dataToExport = events;
      filename = 'events.csv';
    } else if (type === 'logs') {
      dataToExport = logs;
      filename = 'logs.csv';
    } else if (type === 'emergency') {
      dataToExport = emergencies;
      filename = 'emergencies.csv';
    }
    
    if (dataToExport.length === 0) {
      alert('내보낼 데이터가 없습니다');
      return;
    }
    
    // Create CSV
    const headers = Object.keys(dataToExport[0]).join(',');
    const rows = dataToExport.map(item => 
      Object.values(item).map(value => 
        typeof value === 'string' ? `"${value.replace(/"/g, '""')}"` : value
      ).join(',')
    ).join('\n');
    const csv = `${headers}\n${rows}`;
    
    // Create download link
    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.setAttribute('download', filename);
    link.click();
  };

  const getEventTypeColor = (type) => {
    switch (type) {
      case 'EMERGENCY':
        return 'bg-red-100 text-red-800';
      case 'WELCOME':
      case 'CALL':
        return 'bg-blue-100 text-blue-800';
      case 'BIRTHDAY':
        return 'bg-green-100 text-green-800';
      case 'CLEANING':
        return 'bg-purple-100 text-purple-800';
      default:
        return 'bg-gray-100 text-gray-800';
    }
  };

  const getLogLevelColor = (level) => {
    switch (level?.toUpperCase()) {
      case 'ERROR':
        return 'bg-red-100 text-red-800';
      case 'WARNING':
        return 'bg-yellow-100 text-yellow-800';
      case 'INFO':
        return 'bg-blue-100 text-blue-800';
      case 'DEBUG':
        return 'bg-gray-100 text-gray-800';
      default:
        return 'bg-gray-100 text-gray-800';
    }
  };

  return (
    <Layout>
      <h1 className="text-2xl font-bold text-gray-800 mb-6">시스템 관리</h1>

      {error && (
        <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6 flex items-center">
          <AlertCircle size={20} className="mr-2" />
          {error}
        </div>
      )}

      <div className="space-y-4">
        {/* Events Panel */}
        <div className="bg-white rounded-lg shadow overflow-hidden">
          <div
            className="flex justify-between items-center p-4 cursor-pointer bg-gray-50"
            onClick={() => handlePanelToggle('events')}
          >
            <h2 className="text-lg font-semibold text-gray-800">이벤트</h2>
            <div className="flex items-center">
              <button
                onClick={(e) => {
                  e.stopPropagation();
                  exportData('events');
                }}
                className="mr-4 text-blue-600 hover:text-blue-800"
                aria-label="Export data"
              >
                <Download size={18} />
              </button>
              {expandedPanel === 'events' ? <ChevronUp size={20} /> : <ChevronDown size={20} />}
            </div>
          </div>
          
          {expandedPanel === 'events' && (
            <div className="p-4">
              {isLoading ? (
                <div className="flex items-center justify-center h-64">
                  <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
                </div>
              ) : events.length > 0 ? (
                <div className="overflow-x-auto">
                  <table className="min-w-full divide-y divide-gray-200">
                    <thead className="bg-gray-50">
                      <tr>
                        <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          ID
                        </th>
                        <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          유형
                        </th>
                        <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          대상
                        </th>
                        <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          설명
                        </th>
                        <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          시간
                        </th>
                      </tr>
                    </thead>
                    <tbody className="bg-white divide-y divide-gray-200">
                      {events.map((event) => (
                        <tr key={event.id} className="hover:bg-gray-50">
                          <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                            {event.id}
                          </td>
                          <td className="px-6 py-4 whitespace-nowrap">
                            <span className={`px-2 py-1 inline-flex text-xs leading-5 font-semibold rounded-full ${getEventTypeColor(event.type)}`}>
                              {event.type}
                            </span>
                          </td>
                          <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                            {event.related_entity_type}/{event.related_entity_id}
                          </td>
                          <td className="px-6 py-4 text-sm text-gray-500 max-w-md truncate">
                            {event.description}
                          </td>
                          <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                            {formatTimestamp(event.timestamp)}
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              ) : (
                <div className="text-center text-gray-500 py-8">
                  이벤트 데이터가 없습니다
                </div>
              )}
            </div>
          )}
        </div>

        {/* Logs Panel */}
        <div className="bg-white rounded-lg shadow overflow-hidden">
          <div
            className="flex justify-between items-center p-4 cursor-pointer bg-gray-50"
            onClick={() => handlePanelToggle('logs')}
          >
            <h2 className="text-lg font-semibold text-gray-800">로그</h2>
            <div className="flex items-center">
              <button
                onClick={(e) => {
                  e.stopPropagation();
                  exportData('logs');
                }}
                className="mr-4 text-blue-600 hover:text-blue-800"
                aria-label="Export data"
              >
                <Download size={18} />
              </button>
              {expandedPanel === 'logs' ? <ChevronUp size={20} /> : <ChevronDown size={20} />}
            </div>
          </div>
          
          {expandedPanel === 'logs' && (
            <div className="p-4">
              <div className="mb-4 flex flex-wrap items-center gap-4">
                <div className="flex items-center">
                  <Filter size={16} className="mr-2 text-gray-500" />
                  <select
                    value={logLevel}
                    onChange={(e) => setLogLevel(e.target.value)}
                    className="px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  >
                    <option value="ALL">모든 레벨</option>
                    <option value="ERROR">ERROR</option>
                    <option value="WARNING">WARNING</option>
                    <option value="INFO">INFO</option>
                    <option value="DEBUG">DEBUG</option>
                  </select>
                </div>
                
                <div className="flex items-center">
                  <span className="mr-2 text-sm text-gray-500">표시 개수</span>
                  <select
                    value={logLimit}
                    onChange={(e) => setLogLimit(parseInt(e.target.value))}
                    className="px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  >
                    <option value="50">50</option>
                    <option value="100">100</option>
                    <option value="200">200</option>
                    <option value="500">500</option>
                  </select>
                </div>
                
                <button
                  onClick={() => fetchData()}
                  className="ml-auto flex items-center text-blue-600 hover:text-blue-800"
                >
                  <RefreshCw size={16} className="mr-1" />
                  새로고침
                </button>
              </div>
              
              {isLoading ? (
                <div className="flex items-center justify-center h-64">
                  <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
                </div>
              ) : logs.length > 0 ? (
                <div className="overflow-x-auto">
                  <table className="min-w-full divide-y divide-gray-200">
                    <thead className="bg-gray-50">
                      <tr>
                        <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          ID
                        </th>
                        <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          레벨
                        </th>
                        <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          메시지
                        </th>
                        <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          시간
                        </th>
                      </tr>
                    </thead>
                    <tbody className="bg-white divide-y divide-gray-200">
                      {logs.map((log) => (
                        <tr key={log.id} className="hover:bg-gray-50">
                          <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                            {log.id}
                          </td>
                          <td className="px-6 py-4 whitespace-nowrap">
                            <span className={`px-2 py-1 inline-flex text-xs leading-5 font-semibold rounded-full ${getLogLevelColor(log.level)}`}>
                              {log.level}
                            </span>
                          </td>
                          <td className="px-6 py-4 text-sm text-gray-500">
                            {log.message}
                          </td>
                          <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                            {formatTimestamp(log.timestamp)}
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              ) : (
                <div className="text-center text-gray-500 py-8">
                  로그 데이터가 없습니다
                </div>
              )}
            </div>
          )}
        </div>

        {/* Emergencies Panel */}
        <div className="bg-white rounded-lg shadow overflow-hidden">
          <div
            className="flex justify-between items-center p-4 cursor-pointer bg-gray-50"
            onClick={() => handlePanelToggle('emergency')}
          >
            <h2 className="text-lg font-semibold text-gray-800">비상 상황</h2>
            <div className="flex items-center">
              <button
                onClick={(e) => {
                  e.stopPropagation();
                  exportData('emergency');
                }}
                className="mr-4 text-blue-600 hover:text-blue-800"
                aria-label="Export data"
              >
                <Download size={18} />
              </button>
              {expandedPanel === 'emergency' ? <ChevronUp size={20} /> : <ChevronDown size={20} />}
            </div>
          </div>
          
          {expandedPanel === 'emergency' && (
            <div className="p-4">
              {isLoading ? (
                <div className="flex items-center justify-center h-64">
                  <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
                </div>
              ) : emergencies.length > 0 ? (
                <div className="space-y-4">
                  {emergencies.map((emergency) => (
                    <div
                      key={emergency.id}
                      className="border border-red-200 rounded-lg p-4 bg-red-50"
                    >
                      <div className="flex justify-between items-start">
                        <div>
                          <div className="font-medium text-red-800">비상 상황 #{emergency.id}</div>
                          <div className="text-sm text-gray-700 mt-1">{emergency.description}</div>
                        </div>
                        <span className="text-sm text-gray-500">
                          {formatTimestamp(emergency.timestamp)}
                        </span>
                      </div>
                      <div className="mt-2 text-sm text-gray-600">
                        대상: {emergency.related_entity_type}/{emergency.related_entity_id}
                      </div>
                    </div>
                  ))}
                </div>
              ) : (
                <div className="text-center text-gray-500 py-8">
                  비상 상황 데이터가 없습니다
                </div>
              )}
            </div>
          )}
        </div>
      </div>
    </Layout>
  );
};

export default SystemPage; 