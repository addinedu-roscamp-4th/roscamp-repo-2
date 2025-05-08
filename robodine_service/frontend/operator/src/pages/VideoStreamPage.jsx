import React, { useState, useEffect, useMemo, useCallback, useRef } from 'react';
import Layout from '../components/Layout';
import { useWebSockets } from '../contexts/WebSocketContext';
import { 
  Video,
  RefreshCw, 
  Clock,
  Calendar,
  Download,
  AlignLeft,
  Filter,
  ExternalLink,
  PlayCircle,
  StopCircle,
  Eye,
  XCircle,
  AlertTriangle,
  Wifi,
  WifiOff,
  RotateCw
} from 'lucide-react';

const VideoStreamPage = () => {
  const [activeTab, setActiveTab] = useState('ALL');
  const [selectedVideo, setSelectedVideo] = useState(null);
  const [isPlayerOpen, setIsPlayerOpen] = useState(false);
  const { data, errors, connected, refreshTopic } = useWebSockets();
  const [isLoading, setIsLoading] = useState(true);
  const [lastRefreshTime, setLastRefreshTime] = useState(0);
  const [errorMessage, setErrorMessage] = useState(null);
  const [videoLoadError, setVideoLoadError] = useState(false);
  const refreshAttempts = useRef(0);
  const maxRefreshAttempts = 3;
  const refreshTimeoutRef = useRef(null);
  
  // WebSocket 연결 상태를 로그에서 확인
  useEffect(() => {
    try {
      const logKey = `ws_log_video_streams`;
      const logs = JSON.parse(localStorage.getItem(logKey) || '[]');
      if (logs.length > 0) {
        console.debug('비디오 스트림 웹소켓 로그:', logs);
      }
    } catch (e) {
      // 무시
    }
  }, []);
  
  // 비디오 스트림 데이터 - 에러 방어 로직 추가
  const videoStreams = useMemo(() => {
    // 에러 없이 데이터가 있는 경우만 사용
    if (Array.isArray(data.video_streams) && data.video_streams.length > 0) {
      return data.video_streams;
    }
    
    // 로컬 스토리지에 캐시된 데이터가 있으면 사용
    const cachedData = localStorage.getItem('cachedVideoStreams');
    if (cachedData) {
      try {
        return JSON.parse(cachedData);
      } catch (e) {
        console.error('캐시된 비디오 스트림 데이터 파싱 오류:', e);
      }
    }
    
    return [];
  }, [data.video_streams]);

  // 비디오 스트림 데이터 캐싱
  useEffect(() => {
    if (Array.isArray(data.video_streams) && data.video_streams.length > 0) {
      localStorage.setItem('cachedVideoStreams', JSON.stringify(data.video_streams));
    }
  }, [data.video_streams]);

  // 스트림 새로고침 핸들러 - 디바운스 및 오류 처리 개선
  const handleRefreshStreams = useCallback(() => {
    const now = Date.now();
    const DEBOUNCE_TIME = 3000; // 3초 디바운스
    
    if (now - lastRefreshTime < DEBOUNCE_TIME) {
      return; // 너무 빠른 재요청 방지
    }
    
    setIsLoading(true);
    setLastRefreshTime(now);
    
    try {
      // 최대 재시도 횟수 초과 시 일시적으로 새로고침 중단
      if (refreshAttempts.current >= maxRefreshAttempts) {
        setErrorMessage('연결 오류가 지속됩니다. 서버 상태를 확인해주세요.');
        setIsLoading(false);
        
        // 30초 후 재시도 카운터 초기화
        if (refreshTimeoutRef.current) {
          clearTimeout(refreshTimeoutRef.current);
        }
        
        refreshTimeoutRef.current = setTimeout(() => {
          refreshAttempts.current = 0;
          setErrorMessage(null);
        }, 30000);
        
        return;
      }
      
      // WebSocket 컨텍스트의 refreshTopic 호출
      refreshTopic('video_streams');
      
      // 웹소켓 연결 성공 시 오류 초기화
      if (connected.video_streams) {
        setErrorMessage(null);
        refreshAttempts.current = 0;
      }
    } catch (err) {
      console.error('스트림 데이터 요청 실패:', err);
      setErrorMessage('데이터 요청 중 오류가 발생했습니다. 잠시 후 다시 시도해주세요.');
      refreshAttempts.current++;
    }
    
    // 일정 시간 후 로딩 상태 해제 (데이터가 오지 않더라도)
    const loadingTimeout = setTimeout(() => {
      setIsLoading(false);
    }, 3000);
    
    return () => clearTimeout(loadingTimeout);
  }, [refreshTopic, lastRefreshTime, connected.video_streams]);

  // 웹소켓 연결 상태 모니터링
  useEffect(() => {
    if (!connected.video_streams) {
      // 연결이 끊어진 경우, WebSocketContext에서 자동으로 재연결 시도
      if (!errorMessage || !errorMessage.includes('연결')) {
        setErrorMessage('비디오 스트림 서버에 연결되지 않았습니다. 자동으로 재연결을 시도합니다...');
      }
    } else {
      // 연결이 복구된 경우 
      if (errorMessage && errorMessage.includes('연결')) {
        setErrorMessage(null);
        refreshAttempts.current = 0;
        
        // 연결이 복구되면 자동으로 데이터 갱신
        if (Date.now() - lastRefreshTime > 5000) {
          handleRefreshStreams();
        }
      }
    }
  }, [connected.video_streams, errorMessage, handleRefreshStreams, lastRefreshTime]);
  
  // 페이지 초기 로드 및 WebSocket 오류 처리
  useEffect(() => {
    // WebSocketContext에서 보고된 오류 처리
    if (errors.video_streams) {
      setErrorMessage(`서버 연결 오류: ${errors.video_streams}`);
    }
  }, [errors.video_streams]);
  
  // 비디오 로드 오류 처리
  const handleVideoError = () => {
    setVideoLoadError(true);
  };

  // 최초 접속 시 데이터 로드 - 타임아웃 및 에러 처리 개선
  useEffect(() => {
    // 최초 1회 로드
    if (!isLoading && videoStreams.length === 0) {
      handleRefreshStreams();
    }
    
    // 자동 갱신 주기 설정 (60초)
    const refreshInterval = setInterval(() => {
      // 재시도 횟수가 최대치를 초과하지 않은 경우에만 자동 갱신
      if (connected.video_streams && refreshAttempts.current < maxRefreshAttempts) {
        handleRefreshStreams();
      }
    }, 60000);
    
    // 연결 되지 않은 경우 - 초기 타임아웃 처리
    const initialConnectionTimer = setTimeout(() => {
      if (!connected.video_streams && videoStreams.length === 0) {
        setIsLoading(false);
        setErrorMessage('데이터 서버에 연결할 수 없습니다. 네트워크 상태를 확인하세요.');
      }
    }, 5000);
    
    return () => {
      clearInterval(refreshInterval);
      clearTimeout(initialConnectionTimer);
      if (refreshTimeoutRef.current) {
        clearTimeout(refreshTimeoutRef.current);
      }
    };
  }, [connected.video_streams, handleRefreshStreams, isLoading, videoStreams.length]);

  // 언마운트 시 타이머 정리
  useEffect(() => {
    return () => {
      if (refreshTimeoutRef.current) {
        clearTimeout(refreshTimeoutRef.current);
      }
    };
  }, []);

  // 페이지 새로고침
  const handlePageRefresh = () => {
    window.location.reload();
  };

  // 고유한 source_type 목록 추출
  const sourceTypes = useMemo(() => {
    const types = new Set(videoStreams.map(stream => stream.source_type));
    return ['ALL', ...Array.from(types)];
  }, [videoStreams]);

  // 활성 탭에 따라 필터링된 스트림
  const filteredStreams = useMemo(() => {
    if (!videoStreams.length) return [];
    
    return activeTab === 'ALL' 
      ? videoStreams 
      : videoStreams.filter(stream => stream.source_type === activeTab);
  }, [videoStreams, activeTab]);

  // 비디오 선택 핸들러
  const handleVideoSelect = (video) => {
    setSelectedVideo(video);
    setIsPlayerOpen(true);
    setVideoLoadError(false); // 새 비디오 선택 시 오류 상태 초기화
  };

  // 비디오 플레이어 닫기
  const handleClosePlayer = () => {
    setIsPlayerOpen(false);
    setSelectedVideo(null);
    setVideoLoadError(false);
  };

  // 날짜 포맷팅
  const formatDate = (dateString) => {
    if (!dateString) return '-';
    const date = new Date(dateString);
    return new Intl.DateTimeFormat('ko-KR', {
      year: 'numeric',
      month: '2-digit',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit',
      hour12: false
    }).format(date);
  };

  // 상태 표시 태그
  const getStatusBadge = (status) => {
    const statusConfig = {
      ACTIVE: { color: 'bg-green-100 text-green-800 border-green-200', icon: <PlayCircle size={14} className="mr-1" /> },
      INACTIVE: { color: 'bg-gray-100 text-gray-800 border-gray-200', icon: <StopCircle size={14} className="mr-1" /> },
      ERROR: { color: 'bg-red-100 text-red-800 border-red-200', icon: <XCircle size={14} className="mr-1" /> }
    };

    const config = statusConfig[status] || statusConfig.INACTIVE;
    
    return (
      <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium border ${config.color}`}>
        {config.icon}
        {status}
      </span>
    );
  };

  return (
    <Layout>
      <div className="container mx-auto p-4 sm:p-6">
        <div className="mb-6 flex justify-between items-center">
          <div>
            <h1 className="text-2xl font-bold text-gray-800 flex items-center">
              <Video className="mr-2" />
              영상 스트림 관리
            </h1>
            <div className="flex items-center gap-2 text-gray-600">
              <p>모든 영상 스트림을 확인하고 관리합니다.</p>
              {connected.video_streams ? (
                <span className="flex items-center text-green-600 text-xs bg-green-50 px-2 py-0.5 rounded-full">
                  <Wifi size={12} className="mr-1" />
                  연결됨
                </span>
              ) : (
                <span className="flex items-center text-red-600 text-xs bg-red-50 px-2 py-0.5 rounded-full">
                  <WifiOff size={12} className="mr-1" />
                  연결 끊김
                </span>
              )}
            </div>
          </div>
          <div className="flex gap-2">
            <button
              onClick={handlePageRefresh}
              className="p-2 bg-white border border-gray-300 rounded-md hover:bg-gray-50 flex items-center gap-2"
              title="페이지 새로고침"
            >
              <RotateCw size={16} />
            </button>
            <button
              onClick={handleRefreshStreams}
              className="p-2 bg-white border border-gray-300 rounded-md hover:bg-gray-50 flex items-center gap-2"
              disabled={isLoading || Date.now() - lastRefreshTime < 3000 || refreshAttempts.current >= maxRefreshAttempts}
            >
              <RefreshCw size={16} className={isLoading ? "animate-spin" : ""} />
              {isLoading ? "로딩 중" : "새로고침"}
            </button>
          </div>
        </div>

        {/* 오류 메시지 표시 */}
        {errorMessage && (
          <div className="mb-6 bg-yellow-50 border border-yellow-200 rounded-md p-4 flex items-start">
            <AlertTriangle className="text-yellow-500 mr-3 mt-0.5 flex-shrink-0" size={20} />
            <div>
              <h3 className="font-medium text-yellow-700">연결 상태 알림</h3>
              <p className="text-yellow-600 text-sm mt-1">{errorMessage}</p>
              {(refreshAttempts.current >= maxRefreshAttempts || errors.video_streams?.includes('최대 시도 횟수')) && (
                <div className="mt-2">
                  <p className="text-yellow-600 text-sm">
                    연결 재시도 횟수가 초과되었습니다. 페이지를 새로고침 하거나 잠시 후 다시 시도해주세요.
                  </p>
                  <button 
                    onClick={handlePageRefresh}
                    className="mt-2 text-sm bg-yellow-100 hover:bg-yellow-200 text-yellow-800 px-3 py-1 rounded flex items-center gap-1 w-fit"
                  >
                    <RotateCw size={12} />
                    페이지 새로고침
                  </button>
                </div>
              )}
            </div>
          </div>
        )}

        {/* 탭 네비게이션 */}
        <div className="mb-6 overflow-x-auto">
          <div className="flex space-x-1 border-b border-gray-200">
            {sourceTypes.map((type) => (
              <button
                key={type}
                className={`px-4 py-2 font-medium text-sm ${
                  activeTab === type
                    ? 'text-blue-600 border-b-2 border-blue-600'
                    : 'text-gray-500 hover:text-gray-700 hover:border-gray-300'
                }`}
                onClick={() => setActiveTab(type)}
              >
                {type}
              </button>
            ))}
          </div>
        </div>

        {/* 비디오 스트림 그리드 */}
        {isLoading && videoStreams.length === 0 ? (
          <div className="bg-white rounded-lg shadow p-6 text-center">
            <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500 mx-auto"></div>
            <p className="mt-4 text-gray-600">영상 스트림 정보를 불러오는 중...</p>
          </div>
        ) : errors.video_streams && videoStreams.length === 0 ? (
          <div className="bg-white rounded-lg shadow p-6 text-center">
            <XCircle className="mx-auto h-12 w-12 text-red-400 mb-4" />
            <h3 className="text-lg font-medium text-gray-900">오류가 발생했습니다</h3>
            <p className="mt-2 text-gray-500">{errors.video_streams}</p>
            <button 
              onClick={handlePageRefresh}
              className="mt-4 bg-gray-100 hover:bg-gray-200 text-gray-800 px-4 py-2 rounded flex items-center gap-2 mx-auto"
            >
              <RotateCw size={16} />
              페이지 새로고침
            </button>
          </div>
        ) : filteredStreams.length === 0 ? (
          <div className="bg-white rounded-lg shadow p-6 text-center">
            <Video className="mx-auto h-12 w-12 text-gray-400 mb-4" />
            <h3 className="text-lg font-medium text-gray-900">영상 스트림이 없습니다</h3>
            <p className="mt-2 text-gray-500">
              {activeTab !== 'ALL' 
                ? `${activeTab} 타입의 스트림이 없습니다.` 
                : '등록된 영상 스트림이 없습니다.'}
            </p>
          </div>
        ) : (
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
            {filteredStreams.map((stream) => (
              <div key={stream.id} className="bg-white rounded-lg shadow overflow-hidden">
                <div className="p-4 border-b border-gray-200">
                  <div className="flex justify-between items-start">
                    <h3 className="font-medium text-gray-900 truncate flex items-center">
                      <Video size={18} className="mr-2 text-gray-500" />
                      {stream.source_type} #{stream.source_id}
                    </h3>
                    {getStatusBadge(stream.status)}
                  </div>
                </div>
                
                <div className="px-4 py-3 bg-gray-50">
                  <div className="grid grid-cols-1 gap-2">
                    <div className="flex items-start">
                      <AlignLeft size={16} className="mr-2 mt-0.5 text-gray-500" />
                      <div>
                        <p className="text-xs text-gray-500">스트림 ID</p>
                        <p className="text-sm font-medium">{stream.id}</p>
                      </div>
                    </div>
                    
                    <div className="flex items-start">
                      <Clock size={16} className="mr-2 mt-0.5 text-gray-500" />
                      <div>
                        <p className="text-xs text-gray-500">마지막 확인</p>
                        <p className="text-sm">{formatDate(stream.last_checked)}</p>
                      </div>
                    </div>
                    
                    {stream.recording_path && (
                      <div className="flex items-start">
                        <Calendar size={16} className="mr-2 mt-0.5 text-gray-500" />
                        <div>
                          <p className="text-xs text-gray-500">녹화 기간</p>
                          <p className="text-sm">
                            {formatDate(stream.recording_started_at)} ~ 
                            <br />
                            {formatDate(stream.recording_ended_at)}
                          </p>
                        </div>
                      </div>
                    )}
                  </div>
                </div>
                
                <div className="p-4 bg-white flex justify-between items-center">
                  {stream.recording_path ? (
                    <button
                      onClick={() => handleVideoSelect(stream)}
                      className="text-blue-600 hover:text-blue-800 font-medium text-sm flex items-center"
                    >
                      <Eye size={16} className="mr-1" />
                      녹화 영상 보기
                    </button>
                  ) : (
                    <span className="text-gray-400 text-sm">녹화 영상 없음</span>
                  )}
                  
                  {stream.url && (
                    <a
                      href={stream.url}
                      target="_blank"
                      rel="noopener noreferrer"
                      className="text-gray-600 hover:text-gray-800 text-sm flex items-center"
                    >
                      <ExternalLink size={16} className="mr-1" />
                      원본 스트림
                    </a>
                  )}
                </div>
              </div>
            ))}
          </div>
        )}
        
        {/* 비디오 플레이어 모달 */}
        {isPlayerOpen && selectedVideo && (
          <div className="fixed inset-0 bg-gray-600 bg-opacity-50 flex items-center justify-center z-50">
            <div className="bg-white rounded-lg shadow-xl max-w-4xl w-full mx-4 max-h-[90vh] overflow-hidden flex flex-col">
              <div className="border-b px-6 py-4 flex items-center justify-between">
                <h2 className="text-xl font-semibold text-gray-800 flex items-center">
                  <Video className="mr-2" size={20} />
                  {selectedVideo.source_type} #{selectedVideo.source_id} 녹화 영상
                </h2>
                <button 
                  onClick={handleClosePlayer}
                  className="text-gray-400 hover:text-gray-500"
                >
                  <XCircle size={20} />
                </button>
              </div>
              
              <div className="p-4 flex-grow overflow-auto">
                {videoLoadError ? (
                  <div className="aspect-video bg-gray-100 flex items-center justify-center rounded overflow-hidden">
                    <div className="text-center p-6">
                      <AlertTriangle className="mx-auto h-12 w-12 text-red-400 mb-2" />
                      <h3 className="text-lg font-medium text-gray-900">영상을 불러올 수 없습니다</h3>
                      <p className="mt-2 text-gray-500">녹화 영상 파일에 접근할 수 없습니다. 서버 관리자에게 문의하세요.</p>
                    </div>
                  </div>
                ) : (
                  <div className="aspect-video bg-black relative rounded overflow-hidden">
                    <video
                      className="absolute inset-0 w-full h-full object-contain"
                      controls
                      autoPlay
                      src={`http://localhost:8000/${selectedVideo.recording_path}`}
                      onError={handleVideoError}
                    />
                  </div>
                )}

                <div className="mt-4 flex items-center">
                  <p className="text-sm text-gray-500 mr-2">영상 소스 URL:</p>
                  <a
                    href={selectedVideo.url}
                    target="_blank"
                    rel="noopener noreferrer"
                    className="text-blue-600 hover:text-blue-800 text-sm"
                  >
                    {`http://localhost:8000/${selectedVideo.recording_path}`}
                  </a>
                </div>
                
                <div className="mt-4 grid grid-cols-1 md:grid-cols-2 gap-4">
                  <div>
                    <p className="text-sm text-gray-500 mb-1">녹화 시작 시간</p>
                    <p className="text-lg font-medium">{formatDate(selectedVideo.recording_started_at)}</p>
                  </div>
                  <div>
                    <p className="text-sm text-gray-500 mb-1">녹화 종료 시간</p>
                    <p className="text-lg font-medium">{formatDate(selectedVideo.recording_ended_at)}</p>
                  </div>
                </div>
                
                <div className="mt-4 border-t pt-4">
                  <a
                    href={`/${selectedVideo.recording_path}`}
                    download
                    className="inline-flex items-center px-4 py-2 bg-blue-600 text-white rounded hover:bg-blue-700"
                  >
                    <Download size={16} className="mr-2" />
                    영상 다운로드
                  </a>
                </div>
              </div>
            </div>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default VideoStreamPage;
