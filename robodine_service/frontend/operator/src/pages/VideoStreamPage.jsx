import React, { useState, useEffect, useCallback, useMemo, useRef } from 'react';
import { Video, Camera, Settings, RefreshCw, AlertTriangle } from 'react-feather';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useWebSockets } from '../contexts/WebSocketContext';

// 비디오 스트림 카드 컴포넌트
const VideoStreamCard = ({ stream, onClick }) => {
  // 스트림 상태 확인 (대소문자 고려)
  const streamActive = stream?.status?.toUpperCase() === 'ACTIVE';
  
  return (
    <div 
      className="bg-white p-4 rounded-lg shadow cursor-pointer hover:shadow-md transition-shadow"
      onClick={() => onClick(stream.id)}
    >
      <div className="mb-2 flex justify-between items-center">
        <h3 className="font-medium text-gray-800">{getStreamName(stream)}</h3>
        <div className={`px-2 py-1 rounded-full text-xs flex items-center ${
          streamActive 
            ? 'bg-green-100 text-green-800' 
            : 'bg-red-100 text-red-800'
        }`}>
          <span className={`w-2 h-2 rounded-full mr-1 ${streamActive ? 'bg-green-500' : 'bg-red-500'}`}></span>
          {streamActive ? '활성' : '비활성'}
        </div>
      </div>
      
      <div className="h-32 bg-gray-200 rounded mb-2 flex items-center justify-center">
        <Camera size={32} className="text-gray-400" />
      </div>
      
      <p className="text-xs text-gray-600 mb-1">{getStreamDescription(stream)}</p>
      <p className="text-xs text-gray-500">
        마지막 확인: {stream.last_checked ? new Date(stream.last_checked).toLocaleString('ko-KR') : '정보 없음'}
      </p>
    </div>
  );
};

// 비디오 스트림 상세 보기 컴포넌트
const VideoStreamDetail = ({ stream, onClose }) => {
  // 녹화 경로 표시
  const recordingInfo = useMemo(() => {
    if (!stream) return null;
    
    return {
      path: stream.recording_path || '녹화 경로 없음',
      started: stream.recording_started_at ? new Date(stream.recording_started_at).toLocaleString('ko-KR') : '-',
      ended: stream.recording_ended_at ? new Date(stream.recording_ended_at).toLocaleString('ko-KR') : '-',
    };
  }, [stream]);
  
  if (!stream) return null;
  
  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div className="bg-white rounded-lg shadow-lg max-w-2xl w-full mx-4">
        <div className="flex justify-between items-center p-4 border-b">
          <h2 className="text-xl font-medium">{getStreamName(stream)}</h2>
          <button 
            className="text-gray-500 hover:text-gray-700"
            onClick={onClose}
          >
            <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth="2" d="M6 18L18 6M6 6l12 12" />
            </svg>
          </button>
        </div>
        
        <div className="p-4">
          <div className="mb-6">
            <div className="bg-gray-200 h-64 rounded-lg flex items-center justify-center mb-4">
              <Camera size={48} className="text-gray-400" />
              <p className="ml-3 text-gray-500">녹화된 영상 정보만 확인 가능합니다</p>
            </div>
          </div>
          
          <div className="grid grid-cols-2 gap-4 mb-4">
            <div>
              <h3 className="text-sm font-medium text-gray-700 mb-1">소스 타입</h3>
              <p className="text-gray-900">{stream.source_type || '-'}</p>
            </div>
            <div>
              <h3 className="text-sm font-medium text-gray-700 mb-1">소스 ID</h3>
              <p className="text-gray-900">{stream.source_id || '-'}</p>
            </div>
            <div>
              <h3 className="text-sm font-medium text-gray-700 mb-1">상태</h3>
              <p className={`px-2 py-1 rounded-full text-xs inline-flex items-center ${
                stream.status?.toUpperCase() === 'ACTIVE' 
                  ? 'bg-green-100 text-green-800' 
                  : 'bg-red-100 text-red-800'
              }`}>
                <span className={`w-2 h-2 rounded-full mr-1 ${stream.status?.toUpperCase() === 'ACTIVE' ? 'bg-green-500' : 'bg-red-500'}`}></span>
                {stream.status?.toUpperCase() === 'ACTIVE' ? '활성' : '비활성'}
              </p>
            </div>
            <div>
              <h3 className="text-sm font-medium text-gray-700 mb-1">마지막 확인</h3>
              <p className="text-gray-900">{stream.last_checked ? new Date(stream.last_checked).toLocaleString('ko-KR') : '정보 없음'}</p>
            </div>
          </div>
          
          <div className="bg-gray-50 p-4 rounded-lg mb-4">
            <h3 className="text-sm font-medium text-gray-700 mb-2">녹화 정보</h3>
            <div className="grid grid-cols-1 gap-2">
              <div className="flex justify-between">
                <span className="text-gray-600">저장 경로:</span>
                <span className="text-gray-900 font-mono text-sm">{recordingInfo.path}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-600">녹화 시작:</span>
                <span className="text-gray-900">{recordingInfo.started}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-600">녹화 종료:</span>
                <span className="text-gray-900">{recordingInfo.ended}</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

// 스트림 이름 생성 함수
const getStreamName = (stream) => {
  if (!stream) return '';
  
  const sourceType = stream.source_type || '';
  
  switch (sourceType) {
    case 'GLOBAL_CAM':
      return '매장 전체 카메라';
    case 'COOKBOT':
      return '쿡봇 카메라';
    case 'PINKY':
      return '알바봇 카메라';
    default:
      return `${sourceType} ${stream.source_id || ''}`;
  }
};

// 스트림 설명 생성 함수
const getStreamDescription = (stream) => {
  if (!stream) return '';
  
  const sourceType = stream.source_type || '';
  
  switch (sourceType) {
    case 'GLOBAL_CAM':
      return '매장 전체를 촬영하는 메인 카메라';
    case 'COOKBOT':
      return '조리 로봇 시점 카메라';
    case 'PINKY':
      return '서빙 로봇 시점 카메라';
    default:
      return `${sourceType} 비디오 스트림`;
  }
};

const VideoStreamPage = () => {
  // 스트림 데이터 상태 관리
  const [streams, setStreams] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [selectedStreamId, setSelectedStreamId] = useState(null);
  const [filterType, setFilterType] = useState('ALL');
  const { apiCall } = useAuth();
  
  // 데이터 참조 저장
  const streamsDataRef = useRef([]);
  const [initialLoadDone, setInitialLoadDone] = useState(false);
  const [isManualLoading, setIsManualLoading] = useState(false);
  
  // WebSocket 컨텍스트 사용
  const { data, errors, connected, refreshTopic } = useWebSockets();
  const wsStreams = data.video_streams || [];
  
  // 로딩 상태 통합
  const isDataLoading = !initialLoadDone || isManualLoading;
  const dataError = error || (errors.video_streams ? String(errors.video_streams) : null);
  
  // 선택된 스트림 찾기
  const selectedStream = useMemo(() => {
    if (!selectedStreamId) return null;
    return streams.find(stream => stream.id === selectedStreamId);
  }, [selectedStreamId, streams]);
  
  // 필터링된 스트림 목록
  const filteredStreams = useMemo(() => {
    if (filterType === 'ALL') return streams;
    return streams.filter(stream => stream.source_type === filterType);
  }, [streams, filterType]);
  
  // 필터 옵션
  const filterOptions = useMemo(() => {
    const options = [{ value: 'ALL', label: '모든 카메라' }];
    
    // 스트림 데이터에서 고유한 소스 타입 추출
    const uniqueTypes = [...new Set(streams.map(stream => stream.source_type))];
    
    // 소스 타입별 라벨 추가
    uniqueTypes.forEach(type => {
      if (!type) return;
      
      let label = '';
      switch (type) {
        case 'GLOBAL_CAM':
          label = '매장 전체 카메라';
          break;
        case 'COOKBOT':
          label = '쿡봇 카메라';
          break;
        case 'PINKY':
          label = '알바봇 카메라';
          break;
        default:
          label = type;
      }
      
      options.push({ value: type, label });
    });
    
    return options;
  }, [streams]);
  
  // WebSocket 데이터 처리
  useEffect(() => {
    if (wsStreams && wsStreams.length > 0) {
      try {
        // 데이터 일관성 확인
        const validData = wsStreams.every(stream => 
          stream.id !== undefined && 
          stream.source_type !== undefined
        );
        
        if (!validData) {
          console.error('비디오 스트림 데이터 형식이 유효하지 않습니다:', wsStreams);
          return;
        }
        
        // 첫 로드 시 데이터 저장
        if (!initialLoadDone) {
          streamsDataRef.current = [...wsStreams];
          setStreams(wsStreams);
          setInitialLoadDone(true);
          setIsManualLoading(false);
          setError(null);
          return;
        }
        
        // 기존 데이터와 새 데이터 병합
        const mergedStreams = [...streamsDataRef.current];
        
        // ID 기준으로 맵 생성
        const streamsMap = new Map();
        mergedStreams.forEach(stream => {
          streamsMap.set(stream.id, stream);
        });
        
        // 새 데이터로 맵 업데이트
        wsStreams.forEach(stream => {
          streamsMap.set(stream.id, stream);
        });
        
        // 최종 데이터 배열 생성
        const updatedStreams = Array.from(streamsMap.values());
        
        // 상태 업데이트
        streamsDataRef.current = updatedStreams;
        setStreams(updatedStreams);
        setIsManualLoading(false);
        setError(null);
      } catch (err) {
        console.error('비디오 스트림 데이터 처리 중 오류:', err);
        setError('데이터 처리 중 오류가 발생했습니다.');
        setIsManualLoading(false);
      }
    }
  }, [wsStreams, initialLoadDone]);
  
  // API로 스트림 데이터 가져오기
  const fetchStreams = useCallback(async () => {
    setIsManualLoading(true);
    setError(null);
    try {
      // API 호출
      const data = await apiCall('/api/video-streams');
      
      // 데이터 처리 및 저장
      if (Array.isArray(data) && data.length > 0) {
        streamsDataRef.current = data;
        setStreams(data);
        setInitialLoadDone(true);
      } else {
        console.log('API에서 반환된 비디오 스트림 데이터가 없습니다.');
      }
    } catch (err) {
      console.error('비디오 스트림 정보를 불러올 수 없습니다:', err);
      setError('비디오 스트림 정보를 불러올 수 없습니다');
    } finally {
      setIsManualLoading(false);
    }
  }, [apiCall]);
  
  // 컴포넌트 마운트 시 초기 데이터 로드
  useEffect(() => {
    // WebSocket 연결 갱신
    refreshTopic('video_streams');
    
    // 초기 데이터 로드
    if (!wsStreams || wsStreams.length === 0) {
      fetchStreams();
    }
    
    // 1분마다 자동 새로고침
    const refreshInterval = setInterval(() => {
      refreshTopic('video_streams');
    }, 60000);
    
    return () => clearInterval(refreshInterval);
  }, [refreshTopic, fetchStreams, wsStreams]);
  
  // 스트림 새로고침 핸들러
  const handleRefresh = () => {
    setIsManualLoading(true);
    refreshTopic('video_streams');
    fetchStreams();
  };
  
  // 스트림 선택 핸들러
  const handleSelectStream = (streamId) => {
    setSelectedStreamId(streamId);
  };
  
  // 모달 닫기 핸들러
  const handleCloseDetail = () => {
    setSelectedStreamId(null);
  };

  return (
    <Layout>
      <div className="container mx-auto p-4 sm:p-6">
        <div className="flex justify-between items-center mb-6">
          <div>
            <h1 className="text-2xl font-bold text-gray-800 flex items-center">
              <Video className="text-blue-600 mr-2" size={28} />
              비디오 스트림
            </h1>
            <p className="text-gray-600">카메라별 녹화 정보를 확인합니다.</p>
          </div>
          <button 
            onClick={handleRefresh}
            className="flex items-center px-3 py-2 bg-blue-50 text-blue-600 rounded-md hover:bg-blue-100"
            disabled={isDataLoading}
          >
            <RefreshCw size={16} className={`mr-1 ${isDataLoading ? 'animate-spin' : ''}`} />
            {isDataLoading ? '로딩 중...' : '새로고침'}
          </button>
        </div>

        {/* 필터 선택 */}
        <div className="bg-white rounded-lg shadow p-4 mb-6">
          <div className="flex flex-col sm:flex-row sm:items-center gap-4">
            <div className="font-medium text-gray-700">카메라 유형:</div>
            <div className="flex flex-wrap gap-2">
              {filterOptions.map(option => (
                <button
                  key={option.value}
                  className={`px-3 py-1.5 rounded-full text-sm ${
                    filterType === option.value
                      ? 'bg-blue-100 text-blue-700 border border-blue-300'
                      : 'bg-gray-100 text-gray-700 border border-gray-200 hover:bg-gray-200'
                  }`}
                  onClick={() => setFilterType(option.value)}
                >
                  {option.label}
                </button>
              ))}
            </div>
          </div>
        </div>

        {/* 에러 메시지 */}
        {dataError && (
          <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6 flex items-center">
            <AlertTriangle className="mr-2" size={20} />
            {dataError}
          </div>
        )}

        {/* 컨텐츠 영역 */}
        {isDataLoading ? (
          <div className="flex items-center justify-center h-64">
            <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
          </div>
        ) : filteredStreams.length === 0 ? (
          <div className="bg-white rounded-lg shadow p-8 text-center">
            <Camera size={48} className="mx-auto mb-4 text-gray-300" />
            <h3 className="text-lg font-medium text-gray-900 mb-1">비디오 스트림 없음</h3>
            <p className="text-gray-500">
              {filterType !== 'ALL' 
                ? '선택한 필터에 맞는 비디오 스트림이 없습니다.' 
                : '등록된 비디오 스트림이 없습니다.'}
            </p>
          </div>
        ) : (
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-4">
            {filteredStreams.map(stream => (
              <VideoStreamCard 
                key={stream.id} 
                stream={stream} 
                onClick={handleSelectStream}
              />
            ))}
          </div>
        )}
        
        {/* 스트림 상세 모달 */}
        {selectedStream && (
          <VideoStreamDetail 
            stream={selectedStream}
            onClose={handleCloseDetail}
          />
        )}
      </div>
    </Layout>
  );
};

export default VideoStreamPage; 