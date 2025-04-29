import React, { useState, useEffect } from 'react';
import { AlertOctagon, Clock, MapPin, AlertTriangle } from 'react-feather';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';

const EmergencyPage = () => {
  const [emergencies, setEmergencies] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const { apiCall } = useAuth();

  useEffect(() => {
    const fetchEmergencies = async () => {
      setIsLoading(true);
      setError(null);
      try {
        const data = await apiCall('/api/emergencies');
        setEmergencies(data);
      } catch (err) {
        console.error('Failed to load emergencies:', err);
        setError('비상 상황 정보를 불러올 수 없습니다');
      } finally {
        setIsLoading(false);
      }
    };

    fetchEmergencies();
    // 실시간 업데이트를 위한 폴링
    const intervalId = setInterval(fetchEmergencies, 10000);
    return () => clearInterval(intervalId);
  }, [apiCall]);

  // 타임스탬프 포맷팅
  const formatTimestamp = (timestamp) => {
    return new Date(timestamp).toLocaleString('ko-KR', {
      hour: '2-digit',
      minute: '2-digit',
      month: 'short',
      day: 'numeric',
    });
  };

  const getSeverityColor = (severity) => {
    switch (severity) {
      case 'HIGH':
        return 'bg-red-100 text-red-800 border-red-500';
      case 'MEDIUM':
        return 'bg-orange-100 text-orange-800 border-orange-500';
      case 'LOW':
        return 'bg-yellow-100 text-yellow-800 border-yellow-500';
      default:
        return 'bg-gray-100 text-gray-800 border-gray-500';
    }
  };

  const handleResolve = async (id) => {
    try {
      await apiCall(`/api/emergencies/${id}/resolve`, {
        method: 'PUT',
        body: JSON.stringify({ resolved: true })
      });
      // 목록 새로고침
      setEmergencies(emergencies.map(em => 
        em.id === id ? { ...em, resolved: true } : em
      ));
    } catch (error) {
      console.error(`Failed to resolve emergency ${id}:`, error);
      alert('비상 상황 해결 처리 중 오류가 발생했습니다');
    }
  };

  return (
    <Layout>
      <div className="container mx-auto p-4">
        <h1 className="text-2xl font-bold text-gray-800 mb-6 flex items-center">
          <AlertOctagon className="text-red-600 mr-2" size={28} />
          비상 상황 관리
        </h1>

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
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            {emergencies.length > 0 ? (
              emergencies.map((emergency) => (
                <div 
                  key={emergency.id} 
                  className={`border-l-4 p-4 rounded-lg shadow ${emergency.resolved ? 'bg-gray-50 border-gray-300' : getSeverityColor(emergency.severity)}`}
                >
                  <div className="flex justify-between items-start">
                    <div>
                      <h3 className="font-bold text-lg mb-1">{emergency.type}</h3>
                      <p className="text-gray-700">{emergency.description}</p>
                      <div className="flex items-center text-gray-500 mt-2">
                        <Clock size={16} className="mr-1" />
                        <span className="text-sm">{formatTimestamp(emergency.created_at || emergency.timestamp)}</span>
                      </div>
                      {emergency.location && (
                        <div className="flex items-center text-gray-500 mt-1">
                          <MapPin size={16} className="mr-1" />
                          <span className="text-sm">{emergency.location}</span>
                        </div>
                      )}
                    </div>
                    <div>
                      {emergency.resolved ? (
                        <span className="px-2 py-1 bg-green-100 text-green-800 rounded-full text-xs">해결됨</span>
                      ) : (
                        <button
                          onClick={() => handleResolve(emergency.id)}
                          className="px-3 py-1 bg-blue-600 text-white rounded hover:bg-blue-700 transition-colors"
                        >
                          해결 처리
                        </button>
                      )}
                    </div>
                  </div>
                </div>
              ))
            ) : (
              <div className="col-span-2 bg-gray-50 rounded-lg p-8 text-center text-gray-500">
                <AlertOctagon size={48} className="mx-auto mb-4 text-gray-400" />
                <p className="text-lg">현재 비상 상황이 없습니다</p>
                <p className="text-sm mt-2">모든 시스템이 정상적으로 작동 중입니다</p>
              </div>
            )}
          </div>
        )}
      </div>
    </Layout>
  );
};

export default EmergencyPage; 