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
  RotateCw,
  Search,
  CalendarDays,
  ChevronDown,
  Radio,
  Webcam,
  Cast
} from 'lucide-react';
import JSMpeg from 'jsmpeg-player';
import LiveStreamComponent from '../components/LiveStreamComponent';

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
  const [streamError, setStreamError] = useState(false);
  const [selectedStreamId, setSelectedStreamId] = useState(null);

  
  // 검색 및 필터링을 위한 상태 추가
  const [searchTerm, setSearchTerm] = useState('');
  const [showFilters, setShowFilters] = useState(false);
  const [dateFilter, setDateFilter] = useState({
    type: 'none', // none, year, month, day, hour
    value: null
  });
  
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
  
  // 페이지 로드 시 바로 LIVE 탭으로 설정
  useEffect(() => {
    // 페이지 로드 시 LIVE 탭으로 설정
    setActiveTab('LIVE');
    // 스트림 데이터는 백엔드에서 직접 가져옴
    loadStreams();
  }, []);

  // 스트림 목록 직접 가져오기
  const loadStreams = useCallback(() => {
    setIsLoading(true);
    
    fetch(`http://${window.location.hostname}:8000/streams`)
      .then(response => response.json())
      .then(data => {
        console.log('스트림 데이터 가져옴:', data);
        if (data.streams) {
          setIsLoading(false);
          // 웹캠 스트림이 있으면 자동 선택
          const streams = data.streams;
          if (streams && streams.length > 0) {
            const webcam = streams.find(stream => stream.type === 'webcam');
            if (webcam) {
              console.log('웹캠 스트림 선택:', webcam.id);
              setSelectedStreamId(webcam.id);
            } else if (streams.length > 0) {
              console.log('첫 번째 스트림 선택:', streams[0].id);
              setSelectedStreamId(streams[0].id);
            }
          }
        }
      })
      .catch(err => {
        console.error('스트림 목록 가져오기 오류:', err);
        setIsLoading(false);
        setErrorMessage('스트림 목록을 가져오는 중 오류가 발생했습니다');
      });
  }, []);

  // 비디오 스트림 데이터 - 에러 방어 로직 추가
  const videoStreams = useMemo(() => {
    // 로컬 로딩 중인 경우 빈 배열 반환
    if (isLoading) return [];
    
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
  }, [data.video_streams, isLoading]);

  // 페이지 로드 시 바로 LIVE 탭에서 첫 번째 스트림 자동 선택
  useEffect(() => {
    if (activeTab === 'LIVE' && Array.isArray(videoStreams) && videoStreams.length > 0 && !selectedStreamId) {
      // 웹캠 우선 선택, 없으면 첫 번째 스트림 선택
      const webcam = videoStreams.find(stream => stream.type === 'webcam');
      if (webcam) {
        setSelectedStreamId(webcam.id);
      } else if (videoStreams.length > 0) {
        setSelectedStreamId(videoStreams[0].id);
      }
    }
  }, [activeTab, videoStreams, selectedStreamId]);

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
          loadStreams(); // handleRefreshStreams 대신 loadStreams 호출
        }
      }
    }
  }, [connected.video_streams, errorMessage, lastRefreshTime, loadStreams]);
  
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
    return ['ALL', 'LIVE', ...Array.from(types)];
  }, [videoStreams]);

  // 고유한 연도, 월, 일, 시간 목록 추출
  const dateOptions = useMemo(() => {
    if (!videoStreams.length) return { years: [], months: [], days: [], hours: [] };
    
    const years = new Set();
    const months = new Set();
    const days = new Set();
    const hours = new Set();
    
    videoStreams.forEach(stream => {
      if (stream.recording_started_at) {
        const date = new Date(stream.recording_started_at);
        
        years.add(date.getFullYear());
        
        // 월은 1~12로 표시
        const month = date.getMonth() + 1;
        months.add(month < 10 ? `0${month}` : `${month}`);
        
        // 일은 1~31로 표시
        const day = date.getDate();
        days.add(day < 10 ? `0${day}` : `${day}`);
        
        // 시간은 0~23으로 표시
        const hour = date.getHours();
        hours.add(hour < 10 ? `0${hour}` : `${hour}`);
      }
    });
    
    return {
      years: Array.from(years).sort((a, b) => b - a), // 내림차순 정렬
      months: Array.from(months).sort(),
      days: Array.from(days).sort(),
      hours: Array.from(hours).sort()
    };
  }, [videoStreams]);

  // 활성 탭 및 필터에 따라 필터링된 스트림
  const filteredStreams = useMemo(() => {
    // LIVE 탭이 선택된 경우 빈 배열 반환 (Live 탭은 별도로 처리됨)
    if (activeTab === 'LIVE') {
      return [];
    }
    
    if (!videoStreams.length) return [];
    
    // 타입 필터링
    let filtered = activeTab === 'ALL' 
      ? videoStreams 
      : videoStreams.filter(stream => stream.source_type === activeTab);
    
    // 검색어 필터링 (source_id로 검색)
    if (searchTerm.trim()) {
      filtered = filtered.filter(stream => 
        stream.source_id && stream.source_id.toString().includes(searchTerm.trim())
      );
    }
    
    // 날짜 필터링
    if (dateFilter.type !== 'none' && dateFilter.value) {
      filtered = filtered.filter(stream => {
        if (!stream.recording_started_at) return false;
        
        const date = new Date(stream.recording_started_at);
        
        switch (dateFilter.type) {
          case 'year':
            return date.getFullYear() === parseInt(dateFilter.value);
          case 'month':
            return (date.getMonth() + 1) === parseInt(dateFilter.value);
          case 'day':
            return date.getDate() === parseInt(dateFilter.value);
          case 'hour':
            return date.getHours() === parseInt(dateFilter.value);
          default:
            return true;
        }
      });
    }
    
    return filtered;
  }, [videoStreams, activeTab, searchTerm, dateFilter]);

  // 필터 초기화
  const resetFilters = () => {
    setSearchTerm('');
    setDateFilter({ type: 'none', value: null });
  };

  // 필터 토글
  const toggleFilters = () => {
    setShowFilters(prev => !prev);
  };

  // 날짜 필터 변경 핸들러
  const handleDateFilterChange = (type, value) => {
    setDateFilter({ type, value });
  };

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

  // 스트림 타입 상수 정의
  const STREAM_TYPES = {
    WEBCAM: 'webcam',
    RTSP: 'rtsp'
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

        {/* 검색 및 필터링 컨트롤 - LIVE 탭이 아닐 때만 표시 */}
        {activeTab !== 'LIVE' && (
          <div className="mb-6 bg-white rounded-lg shadow p-4">
            {/* 기존 검색/필터링 UI 유지 */}
            <div className="flex flex-col md:flex-row md:items-center gap-4 mb-4">
              {/* Source ID 검색 */}
              <div className="flex-1">
                <div className="relative rounded-md shadow-sm">
                  <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
                    <Search className="h-5 w-5 text-gray-400" />
                  </div>
                  <input
                    type="text"
                    className="focus:ring-blue-500 focus:border-blue-500 block w-full pl-10 pr-12 sm:text-sm border-gray-300 rounded-md py-2"
                    placeholder="Source ID로 검색"
                    value={searchTerm}
                    onChange={(e) => setSearchTerm(e.target.value)}
                  />
                </div>
              </div>
              
              {/* 필터 토글 버튼 */}
              <div>
                <button
                  onClick={toggleFilters}
                  className="inline-flex items-center px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
                >
                  <Filter className="h-4 w-4 mr-2" />
                  녹화 기간 필터
                  <ChevronDown className={`ml-2 h-4 w-4 transition-transform ${showFilters ? 'rotate-180' : ''}`} />
                </button>
              </div>
              
              {/* 필터 초기화 버튼 */}
              {(searchTerm || dateFilter.type !== 'none') && (
                <button
                  onClick={resetFilters}
                  className="inline-flex items-center px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-red-600 bg-white hover:bg-red-50"
                >
                  필터 초기화
                </button>
              )}
            </div>
            
            {/* 날짜 필터링 컨트롤 */}
            {showFilters && (
              <div className="bg-gray-50 p-4 rounded-md border border-gray-200 mt-2">
                <div className="flex items-center mb-3">
                  <CalendarDays className="h-5 w-5 text-gray-500 mr-2" />
                  <span className="text-gray-700 font-medium">녹화 기간 필터링</span>
                </div>
                
                <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
                  {/* 연도 필터 */}
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">연도</label>
                    <select
                      className="mt-1 block w-full pl-3 pr-10 py-2 text-base border-gray-300 focus:outline-none focus:ring-blue-500 focus:border-blue-500 sm:text-sm rounded-md"
                      value={dateFilter.type === 'year' ? dateFilter.value : ''}
                      onChange={(e) => handleDateFilterChange('year', e.target.value || null)}
                    >
                      <option value="">선택 안함</option>
                      {dateOptions.years.map(year => (
                        <option key={year} value={year}>{year}년</option>
                      ))}
                    </select>
                  </div>
                  
                  {/* 월 필터 */}
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">월</label>
                    <select
                      className="mt-1 block w-full pl-3 pr-10 py-2 text-base border-gray-300 focus:outline-none focus:ring-blue-500 focus:border-blue-500 sm:text-sm rounded-md"
                      value={dateFilter.type === 'month' ? dateFilter.value : ''}
                      onChange={(e) => handleDateFilterChange('month', e.target.value || null)}
                    >
                      <option value="">선택 안함</option>
                      {dateOptions.months.map(month => (
                        <option key={month} value={month}>{month}월</option>
                      ))}
                    </select>
                  </div>
                  
                  {/* 일 필터 */}
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">일</label>
                    <select
                      className="mt-1 block w-full pl-3 pr-10 py-2 text-base border-gray-300 focus:outline-none focus:ring-blue-500 focus:border-blue-500 sm:text-sm rounded-md"
                      value={dateFilter.type === 'day' ? dateFilter.value : ''}
                      onChange={(e) => handleDateFilterChange('day', e.target.value || null)}
                    >
                      <option value="">선택 안함</option>
                      {dateOptions.days.map(day => (
                        <option key={day} value={day}>{day}일</option>
                      ))}
                    </select>
                  </div>
                  
                  {/* 시간 필터 */}
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">시간</label>
                    <select
                      className="mt-1 block w-full pl-3 pr-10 py-2 text-base border-gray-300 focus:outline-none focus:ring-blue-500 focus:border-blue-500 sm:text-sm rounded-md"
                      value={dateFilter.type === 'hour' ? dateFilter.value : ''}
                      onChange={(e) => handleDateFilterChange('hour', e.target.value || null)}
                    >
                      <option value="">선택 안함</option>
                      {dateOptions.hours.map(hour => (
                        <option key={hour} value={hour}>{hour}시</option>
                      ))}
                    </select>
                  </div>
                </div>
                
                <div className="mt-3 text-sm text-gray-500">
                  <p>* 녹화 시작 시간을 기준으로 필터링됩니다.</p>
                </div>
              </div>
            )}
            
            {/* 현재 적용된 필터 표시 */}
            {(searchTerm || dateFilter.type !== 'none') && (
              <div className="mt-3 flex items-center flex-wrap gap-2">
                <span className="text-sm text-gray-500">적용된 필터:</span>
                
                {searchTerm && (
                  <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-blue-100 text-blue-800">
                    Source ID: {searchTerm}
                  </span>
                )}
                
                {dateFilter.type === 'year' && dateFilter.value && (
                  <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">
                    {dateFilter.value}년
                  </span>
                )}
                
                {dateFilter.type === 'month' && dateFilter.value && (
                  <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">
                    {dateFilter.value}월
                  </span>
                )}
                
                {dateFilter.type === 'day' && dateFilter.value && (
                  <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">
                    {dateFilter.value}일
                  </span>
                )}
                
                {dateFilter.type === 'hour' && dateFilter.value && (
                  <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">
                    {dateFilter.value}시
                  </span>
                )}
              </div>
            )}
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
                {type === 'LIVE' ? (
                  <div className="flex items-center">
                    <Radio size={14} className="mr-1 text-red-500 animate-pulse" />
                    {type}
                  </div>
                ) : (
                  type
                )}
              </button>
            ))}
          </div>
        </div>
        {/* LIVE 스트림 컨텐츠 */}
        {activeTab === 'LIVE' && (
          <div className="bg-white rounded-lg shadow p-6">
            <div className="mb-4 flex items-center justify-between">
              <div className="flex items-center">
                <Radio className="h-5 w-5 text-red-500 mr-2 animate-pulse" />
                <h3 className="text-lg font-medium text-gray-900">라이브 스트리밍</h3>
              </div>
              <button 
                onClick={loadStreams}
                className="flex items-center bg-blue-50 text-blue-600 px-3 py-1.5 rounded-md text-sm hover:bg-blue-100"
                disabled={isLoading}
              >
                <RefreshCw size={14} className={isLoading ? "mr-1 animate-spin" : "mr-1"} />
                {isLoading ? "로딩 중..." : "새로고침"}
              </button>
            </div>

            {streamError && (
              <div className="mb-4 p-3 bg-yellow-50 rounded border border-yellow-200 flex items-center">
                <AlertTriangle className="mr-2 text-yellow-600" size={16} />
                <span className="text-sm text-yellow-800">
                  웹캠 스트림에 문제가 발생했습니다. 서버를 확인하세요.
                </span>
              </div>
            )}

            <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
              {/* 웹캠/스트림 목록 */}
              <div className="md:col-span-1 border border-gray-200 rounded-lg p-4">
                <h4 className="font-medium mb-4 flex items-center">
                  <Webcam className="h-5 w-5 text-blue-500 mr-2" />
                  라이브 목록
                </h4>
                
                {isLoading ? (
                  <div className="text-center py-4">
                    <div className="animate-spin rounded-full h-6 w-6 border-t-2 border-b-2 border-blue-500 mx-auto"></div>
                    <p className="mt-2 text-gray-500 text-xs">로딩 중...</p>
                  </div>
                ) : (
                  <div className="space-y-2">
                    {videoStreams && videoStreams.length > 0 ? (
                      videoStreams
                        .filter(stream => stream.type === 'webcam')
                        .map((stream) => (
                          <div 
                            key={stream.id} 
                            className={`border ${selectedStreamId === stream.id ? 'border-blue-500 bg-blue-50' : 'border-gray-200'} rounded-lg p-3 cursor-pointer hover:bg-gray-50`}
                            onClick={() => setSelectedStreamId(stream.id)}
                          >
                            <div className="flex items-center mb-1">
                              <Webcam className="h-4 w-4 text-blue-500 mr-1" />
                              <span className="text-sm font-medium">{stream.display_name || stream.id}</span>
                            </div>
                            <p className="text-xs text-gray-500">
                              디바이스: {stream.path}
                            </p>
                          </div>
                        ))
                    ) : (
                      <div className="text-center py-4">
                        <AlertTriangle className="mx-auto h-5 w-5 text-gray-400 mb-1" />
                        <p className="text-gray-500 text-xs">사용 가능한 웹캠이 없습니다</p>
                      </div>
                    )}
                  </div>
                )}
              </div>
              
              {/* 비디오 플레이어 */}
              <div className="md:col-span-3">
                {selectedStreamId ? (
                  <div className="border border-gray-200 rounded-lg">
                    <div className="p-3 border-b border-gray-200">
                      <h4 className="font-medium flex items-center">
                        <PlayCircle className="h-5 w-5 text-gray-500 mr-2" />
                        {videoStreams && videoStreams.find(s => s.id === selectedStreamId)?.display_name || selectedStreamId}
                      </h4>
                    </div>
                    <div className="aspect-video overflow-hidden bg-black">
                      <LiveStreamComponent 
                        streamId={selectedStreamId} 
                        onError={(err) => {
                          console.error("스트림 에러:", err);
                          setStreamError(true);
                        }} 
                        onConnected={() => setStreamError(false)}
                      />
                    </div>
                  </div>
                ) : (
                  <div className="aspect-video border border-gray-200 rounded-lg flex items-center justify-center bg-gray-50">
                    <div className="text-center p-8">
                      <Webcam className="mx-auto h-12 w-12 text-gray-300 mb-2" />
                      <p className="text-gray-500">왼쪽 목록에서 웹캠을 선택하세요</p>
                    </div>
                  </div>
                )}
              </div>
            </div>
          </div>
        )}
        
        
        {/* 비디오 스트림 그리드 (LIVE 탭이 아닌 경우) */}
        {activeTab !== 'LIVE' && (
          <>
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
                <h3 className="text-lg font-medium text-gray-900">검색 결과가 없습니다</h3>
                <p className="mt-2 text-gray-500">
                  {searchTerm || dateFilter.type !== 'none' 
                    ? '검색 조건에 맞는 영상 스트림이 없습니다.' 
                    : activeTab !== 'ALL' 
                      ? `${activeTab} 타입의 스트림이 없습니다.` 
                      : '등록된 영상 스트림이 없습니다.'}
                </p>
                {(searchTerm || dateFilter.type !== 'none') && (
                  <button
                    onClick={resetFilters}
                    className="mt-4 bg-gray-100 hover:bg-gray-200 text-gray-800 px-4 py-2 rounded flex items-center gap-2 mx-auto"
                  >
                    <Filter size={16} />
                    필터 초기화
                  </button>
                )}
              </div>
            ) : (
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                {filteredStreams.map((stream) => (
                  <div key={stream.id} className="bg-white rounded-lg shadow overflow-hidden">
                    {/* 기존 비디오 스트림 카드 내용 */}
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
          </>
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
                      src={`http://192.168.0.156:8000/${selectedVideo.recording_path}`}
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
                    {`http://192.168.0.156:8000/${selectedVideo.recording_path}`}
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
