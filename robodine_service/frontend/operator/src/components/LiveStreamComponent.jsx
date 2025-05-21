// LiveStreamComponent.jsx
import React, { useRef, useEffect, useState, useCallback } from 'react';
import JSMpeg from 'jsmpeg-player';

const LiveStreamComponent = ({ onError }) => {
  const canvasRef = useRef(null);
  const playerRef = useRef(null);
  const initTimeoutRef = useRef(null);
  const errorTimeoutRef = useRef(null);
  const mountedRef = useRef(true); // 마운트 상태를 추적하는 플래그
  const [loading, setLoading] = useState(true);
  const [reconnecting, setReconnecting] = useState(false);
  const attemptRef = useRef(0); // 시도 횟수 추적
  const initializingRef = useRef(false); // 초기화 중 상태 추적

  // 클린업 함수를 별도 선언하여 재사용
  const cleanupResources = useCallback(() => {
    // 1) 재접속 시도 중단
    if (playerRef.current) {
      try {
        playerRef.current.destroy();
      } catch (e) {
        console.error('플레이어 정리 중 오류:', e);
      }
      playerRef.current = null;
    }
    
    // 2) 타이머 클리어
    clearTimeout(initTimeoutRef.current);
    clearTimeout(errorTimeoutRef.current);
    
    // 3) 초기화 플래그 리셋
    initializingRef.current = false;
  }, []);

  useEffect(() => {
    // 마운트 상태 설정
    mountedRef.current = true;
    
    // 이미 초기화 중이면 리턴
    if (initializingRef.current) return;
    
    // 이전 리소스 정리
    cleanupResources();
    
    // 초기화 시작 플래그 설정
    initializingRef.current = true;
    
    // 초기화 과정 지연 (DOM 렌더링 문제 방지)
    const initTimer = setTimeout(() => {

      // WS URL 설정 - 환경 변수 사용
      // 하드코딩된 값 대신 동적으로 현재 호스트에서 가져오기
      const host = window.location.hostname;
      const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const wsPort = '8000'; // 백엔드 포트
      const wsHost = `${wsProtocol}//${host}:${wsPort}`;
      const wsUrl = `${wsHost}/webcam`;
      
      console.log('스트림 연결 시도:', wsUrl);
      
      // 캔버스 확인
      if (!canvasRef.current || !mountedRef.current) {
        initializingRef.current = false;
        return;
      }

      // 초기 연결 타임아웃
      initTimeoutRef.current = setTimeout(() => {
        if (!mountedRef.current) return;
        
        if (loading) {
          setLoading(false);
          setReconnecting(true);
          onError && onError(new Error('연결 타임아웃'));
        }
        initializingRef.current = false;
      }, 8000);

      try {
        // JSMpeg 플레이어 초기화 전 Canvas2D 렌더러 사용 강제 설정
        // WebGL 완전히 비활성화
        if (typeof window !== 'undefined' && window.JSMpeg) {
          console.log('JSMpeg WebGL 사용 강제 비활성화');
          
          // WebGL 초기화 오류 방지를 위한 추가 대응
          if (window.JSMpeg.Renderer && window.JSMpeg.Renderer.WebGL) {
            // WebGL 지원 감지 함수 오버라이드
            window.JSMpeg.Renderer.WebGL.IsSupported = () => false;
            
            // 기존 WebGL 생성자 수정 (오류 발생 방지)
            const originalWebGLConstructor = window.JSMpeg.Renderer.WebGL;
            window.JSMpeg.Renderer.WebGL = function() {
              console.warn('WebGL 렌더러가 비활성화되었습니다. Canvas2D가 대신 사용됩니다.');
              return new window.JSMpeg.Renderer.Canvas2D(...arguments);
            };
          }
        }
        
        // 플레이어 옵션 설정
        const playerOptions = {
          canvas: canvasRef.current,
          autoplay: true,
          audio: false,
          pauseWhenHidden: false,
          disableGl: true,         // WebGL 비활성화
          disableWebGL: true,      // WebGL 비활성화 (중복 보장)
          renderer: 'Canvas2D',    // Canvas2D 렌더러 명시적으로 지정
          loop: false,             // 루프 비활성화
          videoBufferSize: 128 * 1024 * 1024, // 버퍼 크기 증가
          decodeFirstFrame: true,  // 첫 프레임 즉시 디코딩
          maxAudioLag: 0.0,        // 오디오 지연 최소화
          videoBufferHWM: 0.9,     // 비디오 버퍼 최대 워터마크 설정
          progressive: false,      // 점진적 로딩 비활성화
          throttled: false,        // 렌더링 스로틀 비활성화
          stallTimeout: 0,       // 스톨 타임아웃 비활성화
          pauseWhenHidden: false, // 숨겨진 상태에서 일시 정지 비활성화
          chunkSize: 32 * 1024,    // 청크 크기 감소
          
          // 스레딩 관련 설정
          thread: false,           // 스레드 사용 안함 (렌더링 문제 방지)
          
          // 재연결 설정
          reconnect: true,
          reconnectInterval: 5000,
          connectionTimeout: 60000,
          protocols: [],
          
          // 이벤트 핸들러
          onSourceEstablished: () => {
            if (!mountedRef.current) return;
            
            console.log('스트림 소스 연결됨');
            setLoading(false);
            setReconnecting(false);
            attemptRef.current = 0; // 재연결 시도 횟수 초기화
            clearTimeout(initTimeoutRef.current);
            initializingRef.current = false;
          },
          onSourceCompleted: () => {
            if (!mountedRef.current) return;
            
            console.log('스트림 소스 완료됨');
            initializingRef.current = false;
          },
          onConnectionError: (err) => {
            console.error('스트림 연결 오류:', err);
            // UI 업데이트만, 재접속은 내부 옵션으로 계속 시도
            setLoading(false);
            setReconnecting(true);
            clearTimeout(initTimeoutRef.current);
            initializingRef.current = false;
          },
        };

        // 플레이어 인스턴스 생성 전에 확인
        if (!mountedRef.current || !canvasRef.current) {
          initializingRef.current = false;
          return;
        }
        
        const player = new JSMpeg.Player(wsUrl, playerOptions);
        playerRef.current = player;
      } catch (err) {
        if (!mountedRef.current) return;
        
        console.error('JSMpeg 플레이어 초기화 오류:', err);
        setLoading(false);
        onError && onError(err);
        initializingRef.current = false;
      }
    }, 100); // 약간의 지연을 주어 DOM 렌더링 시간 확보

    // 클린업 함수
    return () => {
      // 1) 마운트 상태 플래그 변경
      mountedRef.current = false;
      
      // 2) 초기화 타이머 클리어
      clearTimeout(initTimer);
      
      // 3) 리소스 정리
      cleanupResources();
    };
  }, [onError, cleanupResources]);

  // 수동으로 새로고침하는 핸들러
  const handleRefresh = useCallback(() => {
    // 1) 기존 스트림 정리
    cleanupResources();
    
    // 2) 상태 초기화
    attemptRef.current = 0;
    
    // 3) 페이지 새로고침
    if (typeof window !== 'undefined') {
      window.location.reload();
    }
  }, [cleanupResources]);

  return (
    <div className="relative bg-black rounded-lg overflow-hidden">
      {loading && (
        <div className="absolute inset-0 flex items-center justify-center bg-gray-900 bg-opacity-75">
          <div className="flex flex-col items-center">
            <div className="animate-spin h-12 w-12 border-t-2 border-b-2 border-blue-500 rounded-full mb-2" />
            <p className="text-white text-sm">스트림 연결 중...</p>
          </div>
        </div>
      )}
      
      {reconnecting && !loading && (
        <div className="absolute inset-0 flex items-center justify-center bg-gray-900 bg-opacity-75">
          <div className="flex flex-col items-center">
            <div className="text-yellow-400 mb-2">⚠️</div>
            <p className="text-white text-sm">스트림 재연결 중...</p>
            <button 
              className="mt-2 px-4 py-1 bg-blue-500 text-white rounded-md text-sm"
              onClick={handleRefresh}
            >
              새로고침
            </button>
          </div>
        </div>
      )}
      
      <canvas ref={canvasRef} className="w-full h-full aspect-video" />
    </div>
  );
};

export default LiveStreamComponent;
