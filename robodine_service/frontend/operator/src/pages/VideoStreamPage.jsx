import React, { useState, useEffect } from 'react';
import { Video, Camera, AlertTriangle, Check, Settings } from 'react-feather';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';

const VideoTab = ({ stream, isActive }) => {
  if (!stream) {
    return (
      <div className="flex flex-col items-center">
        <div className="w-full h-80 rounded-lg bg-black flex items-center justify-center text-gray-500">
          <Camera size={48} />
          <p className="ml-3">비디오 스트림을 불러올 수 없습니다</p>
        </div>
      </div>
    );
  }

  return (
    <div className={`flex-col items-center ${isActive ? 'flex' : 'hidden'}`}>
      <div className="relative w-full">
        <video
          src={stream.url}
          className="w-full h-80 rounded-lg bg-black"
          autoPlay
          controls
          poster="/placeholder-video.jpg"
        />
        
        {/* 상태 표시 */}
        <div className="absolute top-2 right-2 z-10">
          <div className={`flex items-center px-2 py-1 rounded-full text-xs ${
            stream.status === 'active' 
              ? 'bg-green-100 text-green-800' 
              : 'bg-red-100 text-red-800'
          }`}>
            {stream.status === 'active' ? (
              <>
                <Check size={12} className="mr-1" />
                라이브
              </>
            ) : (
              <>
                <AlertTriangle size={12} className="mr-1" />
                오프라인
              </>
            )}
          </div>
        </div>
        
        {/* 이벤트 배지 */}
        {stream.events && stream.events.length > 0 && (
          <div className="absolute bottom-2 left-2 z-10">
            <div className="bg-red-600 text-white px-2 py-1 rounded text-xs">
              {stream.events.length} 개의 이벤트
            </div>
          </div>
        )}
      </div>
      
      {/* 비디오 정보 및 컨트롤 */}
      <div className="w-full p-3 bg-gray-100 rounded-b-lg">
        <div className="flex justify-between items-center">
          <h3 className="font-medium">{stream.name}</h3>
          
          {/* 타임스탬프 선택 */}
          <select 
            className="px-2 py-1 border border-gray-300 rounded text-sm"
            defaultValue="live"
          >
            <option value="live">실시간</option>
            {stream.recordings && stream.recordings.map(rec => (
              <option key={rec.id} value={rec.id}>
                {new Date(rec.timestamp).toLocaleString('ko-KR')}
              </option>
            ))}
          </select>
        </div>
        
        <p className="text-sm text-gray-600 mt-1">{stream.description}</p>
      </div>
    </div>
  );
};

const VideoStreamPage = () => {
  const [activeTab, setActiveTab] = useState(0);
  const [streams, setStreams] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const { apiCall } = useAuth();

  useEffect(() => {
    const fetchStreams = async () => {
      setIsLoading(true);
      setError(null);
      try {
        const data = await apiCall('/api/video-streams');
        setStreams(data);
      } catch (err) {
        console.error('Failed to load video streams:', err);
        setError('비디오 스트림 정보를 불러올 수 없습니다');
        
        // 에러 발생 시 기본 스트림 생성 (UI 테스트용)
        const fallbackStreams = [
          {
            id: 1,
            name: '매장 전체 카메라',
            description: '매장 전체를 촬영하는 메인 카메라',
            url: 'https://example.com/stream1',
            status: 'inactive',
            type: 'global'
          },
          {
            id: 2,
            name: '쿡봇 카메라',
            description: '조리 로봇 시점 카메라',
            url: 'https://example.com/stream2',
            status: 'inactive',
            type: 'cookbot'
          },
          {
            id: 3,
            name: '알바봇 카메라',
            description: '서빙 로봇 시점 카메라',
            url: 'https://example.com/stream3',
            status: 'inactive',
            type: 'albabot'
          }
        ];
        setStreams(fallbackStreams);
      } finally {
        setIsLoading(false);
      }
    };

    fetchStreams();
  }, [apiCall]);

  return (
    <Layout>
      <div className="container mx-auto p-4">
        <div className="flex justify-between items-center mb-6">
          <h1 className="text-2xl font-bold text-gray-800 flex items-center">
            <Video className="text-blue-600 mr-2" size={28} />
            비디오 스트림
          </h1>
          <button className="flex items-center text-gray-600 hover:text-gray-900">
            <Settings size={18} className="mr-1" />
            스트림 설정
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
          <div className="bg-white rounded-lg shadow">
            {/* 탭 네비게이션 */}
            <div className="flex border-b">
              {streams.map((stream, index) => (
                <button
                  key={stream.id}
                  className={`py-3 px-6 text-sm font-medium border-b-2 ${
                    activeTab === index
                      ? 'border-blue-500 text-blue-600'
                      : 'border-transparent text-gray-500 hover:text-gray-700'
                  }`}
                  onClick={() => setActiveTab(index)}
                >
                  {stream.name}
                </button>
              ))}
            </div>
            
            {/* 탭 콘텐츠 */}
            <div className="p-4">
              {streams.map((stream, index) => (
                <VideoTab 
                  key={stream.id} 
                  stream={stream} 
                  isActive={activeTab === index} 
                />
              ))}
              
              {streams.length === 0 && (
                <div className="text-center py-8 text-gray-500">
                  <Camera size={48} className="mx-auto mb-4 opacity-25" />
                  <p className="text-lg">사용 가능한 비디오 스트림이 없습니다</p>
                </div>
              )}
            </div>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default VideoStreamPage; 