// LiveStreamComponent.jsx
import React, { useRef, useEffect, useState, useCallback } from 'react';

// 성능 측정을 위한 클래스
class ClientPerformanceMonitor {
  constructor() {
    this.metrics = {
      connectionStartTime: null,
      offerCreatedTime: null,
      answerReceivedTime: null,
      firstFrameTime: null,
      connectionEstablishedTime: null,
      frameIntervals: [],
      webrtcTiming: []
    };
    this.lastFrameTime = null;
    this.frameCount = 0;
    this.performanceInterval = null;
  }

  startConnection() {
    this.metrics.connectionStartTime = performance.now();
    console.log('🔄 연결 시작 시간 기록');
  }

  recordOfferCreated() {
    this.metrics.offerCreatedTime = performance.now();
    const elapsed = this.metrics.offerCreatedTime - this.metrics.connectionStartTime;
    console.log(`📤 Offer 생성 완료: ${elapsed.toFixed(1)}ms`);
  }

  recordAnswerReceived() {
    this.metrics.answerReceivedTime = performance.now();
    const elapsed = this.metrics.answerReceivedTime - this.metrics.offerCreatedTime;
    console.log(`📥 Answer 수신 완료: ${elapsed.toFixed(1)}ms`);
  }

  recordFirstFrame() {
    if (!this.metrics.firstFrameTime) {
      this.metrics.firstFrameTime = performance.now();
      const totalTime = this.metrics.firstFrameTime - this.metrics.connectionStartTime;
      console.log(`🎥 첫 프레임 수신: ${totalTime.toFixed(1)}ms (총 연결 시간)`);
      
      // 서버에 성능 메트릭 전송
      this.sendPerformanceMetrics();
    }
  }

  recordConnectionEstablished() {
    this.metrics.connectionEstablishedTime = performance.now();
    const elapsed = this.metrics.connectionEstablishedTime - this.metrics.connectionStartTime;
    console.log(`✅ WebRTC 연결 완료: ${elapsed.toFixed(1)}ms`);
  }

  recordFrame() {
    const now = performance.now();
    this.frameCount++;
    
    if (this.lastFrameTime) {
      const interval = now - this.lastFrameTime;
      this.metrics.frameIntervals.push(interval);
      
      // 최근 30개 프레임 간격만 유지
      if (this.metrics.frameIntervals.length > 30) {
        this.metrics.frameIntervals.shift();
      }
      
      // 긴 프레임 간격 감지 (60ms 이상 = 16.7fps 이하)
      if (interval > 60) {
        console.warn(`⚠️ 긴 프레임 간격 감지: ${interval.toFixed(1)}ms`);
      }
    }
    
    this.lastFrameTime = now;
    
    // 30프레임마다 성능 요약 출력
    if (this.frameCount % 30 === 0) {
      this.logPerformanceSummary();
    }
  }

  logPerformanceSummary() {
    if (this.metrics.frameIntervals.length > 1) {
      const avgInterval = this.metrics.frameIntervals.reduce((a, b) => a + b, 0) / this.metrics.frameIntervals.length;
      const avgFps = 1000 / avgInterval;
      const maxInterval = Math.max(...this.metrics.frameIntervals);
      const minInterval = Math.min(...this.metrics.frameIntervals);
      
      console.log('=== 클라이언트 성능 요약 ===');
      console.log(`평균 FPS: ${avgFps.toFixed(1)}`);
      console.log(`평균 프레임 간격: ${avgInterval.toFixed(1)}ms`);
      console.log(`최대 간격: ${maxInterval.toFixed(1)}ms, 최소 간격: ${minInterval.toFixed(1)}ms`);
      console.log(`총 프레임 수: ${this.frameCount}`);
    }
  }

  async sendPerformanceMetrics() {
    try {
      const metrics = {
        connectionStartToOffer: this.metrics.offerCreatedTime - this.metrics.connectionStartTime,
        offerToAnswer: this.metrics.answerReceivedTime - this.metrics.offerCreatedTime,
        answerToFirstFrame: this.metrics.firstFrameTime - this.metrics.answerReceivedTime,
        totalConnectionTime: this.metrics.firstFrameTime - this.metrics.connectionStartTime,
        clientUserAgent: navigator.userAgent,
        timestamp: new Date().toISOString()
      };

      const host = window.location.hostname;
      const port = '8000';
      
      // 성능 메트릭을 서버로 전송
      await fetch(`http://${host}:${port}/client-metrics`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          type: 'webrtc_client_performance',
          metrics: metrics
        })
      });
      
      console.log('📊 성능 메트릭 서버 전송 완료:', metrics);
    } catch (error) {
      console.warn('성능 메트릭 전송 실패:', error);
    }
  }

  startPerformanceMonitoring() {
    // 5초마다 성능 요약 출력
    this.performanceInterval = setInterval(() => {
      if (this.frameCount > 0) {
        this.logPerformanceSummary();
      }
    }, 5000);
  }

  stopPerformanceMonitoring() {
    if (this.performanceInterval) {
      clearInterval(this.performanceInterval);
      this.performanceInterval = null;
    }
  }

  reset() {
    this.metrics = {
      connectionStartTime: null,
      offerCreatedTime: null,
      answerReceivedTime: null,
      firstFrameTime: null,
      connectionEstablishedTime: null,
      frameIntervals: [],
      webrtcTiming: []
    };
    this.lastFrameTime = null;
    this.frameCount = 0;
    this.stopPerformanceMonitoring();
  }
}

const LiveStreamComponent = ({ streamId = 'webcam2', onError, onConnected }) => {
  console.log('LiveStreamComponent 렌더링 시작 - streamId:', streamId);
  
  const videoRef = useRef(null);
  const peerConnectionRef = useRef(null);
  const [loading, setLoading] = useState(true);
  const [reconnecting, setReconnecting] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [streamInfo, setStreamInfo] = useState(null);
  const attemptRef = useRef(0);
  const mountedRef = useRef(true);
  const reconnectTimeoutRef = useRef(null);
  const [errorMessage, setErrorMessage] = useState(null);
  const lastIceConnectedTimeRef = useRef(null);
  // ICE 후보 큐 - 원격 설명이 설정되기 전에 도착한 ICE 후보를 저장
  const iceCandidatesQueueRef = useRef([]);
  const remoteDescriptionSetRef = useRef(false);
  
  // 성능 모니터 인스턴스
  const performanceMonitorRef = useRef(new ClientPerformanceMonitor());
  
  // 중복 요청 방지를 위한 상태 변수들
  const lastOfferTimeRef = useRef(0);
  const isSendingOfferRef = useRef(false);
  const isInitializingRef = useRef(false);
  const MIN_OFFER_INTERVAL = 10000; // 최소 10초 간격
  const [rateLimitMessage, setRateLimitMessage] = useState(null);

  // ICE 서버 설정 - STUN 서버만 사용 (HTTP 환경 호환)
  const iceServers = [
    { urls: 'stun:stun.l.google.com:19302' },
    { urls: 'stun:stun1.l.google.com:19302' }
  ];

  // SDP 기본 검증 함수 (aiortc 생성 SDP는 완전하므로 최소한의 검증만)
  const validateSdp = (sdp) => {
    if (!sdp || typeof sdp !== 'string') {
      throw new Error('유효하지 않은 SDP 형식');
    }
    
    // 필수 SDP 요소 확인
    if (!sdp.includes('v=0') || !sdp.includes('m=')) {
      throw new Error('SDP 필수 요소 누락');
    }
    
    // aiortc가 생성한 완전한 SDP이므로 추가 검증 없이 그대로 사용
    console.log('aiortc 생성 SDP 검증 완료 - 표준 준수 확인됨');
    return sdp;
  };

  // Rate limit 메시지 자동 제거
  useEffect(() => {
    if (rateLimitMessage) {
      const timer = setTimeout(() => {
        if (mountedRef.current) {
          setRateLimitMessage(null);
        }
      }, 15000); // 15초 후 메시지 제거

      return () => clearTimeout(timer);
    }
  }, [rateLimitMessage]);
  

  // 연결 정리 함수
  const cleanupConnection = useCallback(() => {
    try {
      // 성능 모니터링 중지
      performanceMonitorRef.current.stopPerformanceMonitoring();
      
      // WebRTC 연결 정리 - 상태 확인 후 안전하게 종료
      if (peerConnectionRef.current) {
        console.log('연결 정리 중, 현재 상태:', peerConnectionRef.current.signalingState);
        
        // 이벤트 핸들러 제거
        peerConnectionRef.current.onicecandidate = null;
        peerConnectionRef.current.oniceconnectionstatechange = null;
        peerConnectionRef.current.ontrack = null;
        peerConnectionRef.current.onconnectionstatechange = null;
        
        // closed 상태가 아닌 경우에만 close() 호출
        if (peerConnectionRef.current.signalingState !== 'closed') {
          peerConnectionRef.current.close();
          console.log('RTCPeerConnection 정상 종료됨');
        } else {
          console.log('RTCPeerConnection이 이미 closed 상태임');
        }
        
        peerConnectionRef.current = null;
      }
      
      // 요청 상태 초기화
      isSendingOfferRef.current = false;
      isInitializingRef.current = false;
      
      console.log('연결 리소스 정리 완료');
    } catch (err) {
      console.error('연결 정리 중 오류:', err);
      // 오류가 발생해도 참조는 정리
      peerConnectionRef.current = null;
      isSendingOfferRef.current = false;
      isInitializingRef.current = false;
    }
  }, []);

  // API 호출로 스트림 정보 가져오기 
  const fetchStreamInfo = useCallback(async () => {
    if (!mountedRef.current) return null;
    
    try {
      const host = window.location.hostname;
      const port = '8000'; // 백엔드 포트
      const url = `http://${host}:${port}/streams/${streamId}`;
      
      const response = await fetch(url);
      if (!response.ok) {
        throw new Error(`스트림 정보 가져오기 실패: ${response.status}`);
      }
      
      const data = await response.json();
      console.log('스트림 정보 수신:', data.stream);
      
      if (mountedRef.current) {
        setStreamInfo(data.stream);
      }
      
      return data.stream;
    } catch (error) {
      console.error('스트림 정보 가져오기 오류:', error);
      if (mountedRef.current) {
        setErrorMessage(`스트림 정보 가져오기 오류: ${error.message}`);
      }
      return null;
    }
  }, [streamId]);

  // Rate limit 체크 함수
  const checkRateLimit = useCallback(() => {
    const now = Date.now();
    const timeSinceLastOffer = now - lastOfferTimeRef.current;
    
    if (timeSinceLastOffer < MIN_OFFER_INTERVAL) {
      const waitTime = Math.ceil((MIN_OFFER_INTERVAL - timeSinceLastOffer) / 1000);
      console.warn(`Rate limit: ${waitTime}초 후 다시 시도하세요`);
      return false;
    }
    
    return true;
  }, []);

  // HTTP 요청으로 WebRTC offer 전송 (중복 요청 방지 로직 추가)
  const sendOfferViaHttp = useCallback(async (offer) => {
    if (!mountedRef.current) return null;
    
    // 중복 요청 방지
    if (isSendingOfferRef.current) {
      console.warn('이미 offer 전송 중입니다. 중복 요청을 무시합니다.');
      return null;
    }
    
    // Rate limit 체크
    if (!checkRateLimit()) {
      const waitTime = Math.ceil((MIN_OFFER_INTERVAL - (Date.now() - lastOfferTimeRef.current)) / 1000);
      setRateLimitMessage(`너무 빈번한 요청입니다. ${waitTime}초 후 다시 시도하세요.`);
      return null;
    }
    
    isSendingOfferRef.current = true;
    lastOfferTimeRef.current = Date.now();
    
    try {
      const host = window.location.hostname;
      const port = '8000'; // 백엔드 포트
      const url = `http://${host}:${port}/webrtc/offer`;
      
      console.log('서버로 SDP offer 전송 중:', offer.type);
      
      const response = await fetch(url, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          streamId: streamId,
          sdp: offer.sdp,
          clientIp: 'frontend' // 클라이언트 식별자 추가
        }),
      });
      
      if (response.status === 429) {
        const errorText = await response.text();
        console.warn('Rate limit 초과:', errorText);
        setRateLimitMessage('요청이 너무 빈번합니다. 잠시 후 다시 시도하세요.');
        throw new Error(`Rate limit exceeded: ${errorText}`);
      }
      
      if (!response.ok) {
        throw new Error(`Offer 전송 실패: ${response.status}`);
      }
      
      const data = await response.json();
      console.log('서버로부터 SDP answer 수신');
      
      // Rate limit 메시지 제거 (성공 시)
      setRateLimitMessage(null);
      
      // SDP 응답 존재 여부만 확인 (정리는 나중에 처리)
      if (!data.sdp) {
        throw new Error('SDP 응답이 없습니다');
      }
      
      return data;
    } catch (error) {
      console.error('Offer 전송 오류:', error);
      if (mountedRef.current) {
        if (error.message.includes('Rate limit')) {
          // Rate limit 오류는 별도 처리 (이미 메시지 설정됨)
        } else {
          setErrorMessage(`서버 연결 오류: ${error.message}`);
        }
      }
      return null;
    } finally {
      isSendingOfferRef.current = false;
    }
  }, [streamId, checkRateLimit]);

  // HTTP 요청으로 ICE 후보 전송
  const sendIceCandidateViaHttp = useCallback(async (candidate, sessionId) => {
    if (!mountedRef.current || !sessionId) return;
    
    try {
      const host = window.location.hostname;
      const port = '8000'; // 백엔드 포트
      const url = `http://${host}:${port}/webrtc/ice-candidate`;
      
      await fetch(url, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          sessionId: sessionId,
          candidate: candidate
        }),
      });
      
      console.log('ICE 후보 전송됨');
    } catch (error) {
      console.error('ICE 후보 전송 오류:', error);
    }
  }, []);

  // 신호 연결 및 WebRTC 초기화 (중복 초기화 방지 로직 추가)
  const initializeConnection = useCallback(async () => {
    if (!mountedRef.current) return;
    
    // Rate limit 체크 추가 - 초기화 요청이 너무 빈번한 경우 차단
    if (!checkRateLimit()) {
      console.warn('초기화 요청이 너무 빈번합니다.');
      return;
    }
    
    // 중복 초기화 방지
    if (isInitializingRef.current) {
      console.warn('이미 초기화 중입니다. 중복 초기화를 무시합니다.');
      return;
    }
    
    isInitializingRef.current = true;
    
    try {
      setLoading(true);
      setErrorMessage(null);
      attemptRef.current++;

      // 성능 모니터 초기화 및 연결 시작 시간 기록
      performanceMonitorRef.current.reset();
      performanceMonitorRef.current.startConnection();
      performanceMonitorRef.current.startPerformanceMonitoring();

      // 이전 연결 정리
      cleanupConnection();

      console.log(`WebRTC 연결 시도 #${attemptRef.current}: 스트림 ID ${streamId}`);

      // 스트림 정보 가져오기
      const streamInfoData = await fetchStreamInfo();
      if (!streamInfoData && mountedRef.current) {
        setLoading(false);
        setReconnecting(true);
        setErrorMessage("스트림 정보를 가져올 수 없습니다.");
        
        // 재시도 (최대 5회)
        if (attemptRef.current < 5 && mountedRef.current) {
          const backoffTime = Math.min(2000 * Math.pow(2, attemptRef.current - 1), 15000);
          console.log(`${backoffTime/1000}초 후 재시도 (${attemptRef.current}/5)`);
          reconnectTimeoutRef.current = setTimeout(() => {
            if (mountedRef.current) {
              isInitializingRef.current = false; // 재시도 전 플래그 해제
              initializeConnection();
            }
          }, backoffTime);
        } else {
          isInitializingRef.current = false;
        }
        
        return;
      }

      // RTCPeerConnection 설정
      await setupPeerConnection();

    } catch (err) {
      console.error('WebRTC 연결 초기화 오류:', err);
      setLoading(false);
      setErrorMessage(err.message || "연결 초기화 실패");
      onError && onError(err);
    } finally {
      // 성공/실패 관계없이 초기화 플래그 해제
      if (mountedRef.current) {
        setTimeout(() => {
          isInitializingRef.current = false;
        }, 1000); // 1초 후 플래그 해제로 즉시 재시도 방지
      }
    }
  }, [streamId, onError]);

  // RTCPeerConnection 설정
  const setupPeerConnection = useCallback(async () => {
    if (!mountedRef.current) return;

    try {
      // RTCPeerConnection 생성
      const pc = new RTCPeerConnection({ 
        iceServers, 
        sdpSemantics: 'unified-plan',
        // HTTP 환경에서 작동하도록 추가 설정
        iceTransportPolicy: 'all'
      });
      peerConnectionRef.current = pc;

      // ICE 후보 이벤트 처리
      pc.onicecandidate = ({ candidate }) => {
        if (!mountedRef.current) return;
        
        if (candidate && sessionId) {
          console.log('ICE 후보 발견');
          sendIceCandidateViaHttp(candidate, sessionId);
        }
      };

      // ICE 연결 상태 변경 처리
      pc.oniceconnectionstatechange = () => {
        if (!mountedRef.current) return;
        
        console.log('ICE 연결 상태:', pc.iceConnectionState);
        
        if (pc.iceConnectionState === 'connected' || pc.iceConnectionState === 'completed') {
          // 연결 성공 - 성능 측정
          performanceMonitorRef.current.recordConnectionEstablished();
          setLoading(false);
          setReconnecting(false);
          lastIceConnectedTimeRef.current = Date.now();
          onConnected && onConnected();
        }
        else if (pc.iceConnectionState === 'failed' || pc.iceConnectionState === 'disconnected') {
          console.warn('ICE 연결 실패 또는 연결 끊김');
          
          // 연결 상태를 확인하고 실제 연결 종료 여부 확인
          const hasActiveTracks = pc.getTransceivers().some(
            transceiver => transceiver.receiver.track && transceiver.receiver.track.readyState === 'live'
          );
          
          // 활성 트랙이 있으면 일시적인 연결 문제로 간주하고 관찰만 함
          if (hasActiveTracks) {
            console.log('일시적인 ICE 연결 문제, 활성 트랙이 있어 계속 관찰...');
            return;
          }
          
          // 최근에 연결된 적이 있으면 바로 재연결 시도
          const shouldImmediatelyRetry = 
            lastIceConnectedTimeRef.current && 
            (Date.now() - lastIceConnectedTimeRef.current) < 10000;
            
          if (mountedRef.current) {
            setReconnecting(true);
            
            if (shouldImmediatelyRetry) {
              // Rate limit을 고려한 재연결 시도
              const timeSinceLastOffer = Date.now() - lastOfferTimeRef.current;
              const minDelay = Math.max(MIN_OFFER_INTERVAL - timeSinceLastOffer, 2000);
              
              console.log(`ICE 연결이 끊겼습니다. ${minDelay/1000}초 후 재연결 시도...`);
              reconnectTimeoutRef.current = setTimeout(() => {
                if (mountedRef.current) {
                  initializeConnection();
                }
              }, minDelay);
            } else {
              setErrorMessage("ICE 연결 실패");
              
              // Rate limit을 고려한 재연결 지연
              const timeSinceLastOffer = Date.now() - lastOfferTimeRef.current;
              const minDelay = Math.max(MIN_OFFER_INTERVAL - timeSinceLastOffer, 8000);
              
              reconnectTimeoutRef.current = setTimeout(() => {
                if (mountedRef.current) {
                  console.log(`ICE 연결 실패. ${minDelay/1000}초 후 재연결 시도...`);
                  initializeConnection();
                }
              }, minDelay);
            }
          }
        }
      };

      // 트랙 수신 이벤트
      pc.ontrack = (event) => {
        if (!mountedRef.current) return;
        
        console.log('미디어 트랙 수신:', event.track.kind);
        
        if (event.track && event.streams && event.streams[0]) {
          if (videoRef.current) {
            videoRef.current.srcObject = event.streams[0];
            
            // 비디오 프레임 수신 모니터링 설정
            if (event.track.kind === 'video') {
              // 첫 프레임 수신 감지
              const handleFirstFrame = () => {
                performanceMonitorRef.current.recordFirstFrame();
                videoRef.current.removeEventListener('loadeddata', handleFirstFrame);
              };
              
              videoRef.current.addEventListener('loadeddata', handleFirstFrame);
              
              // 프레임 간격 모니터링
              const handleTimeUpdate = () => {
                performanceMonitorRef.current.recordFrame();
              };
              
              videoRef.current.addEventListener('timeupdate', handleTimeUpdate);
              
              event.track.onunmute = () => {
                console.log('비디오 트랙 활성화됨');
                performanceMonitorRef.current.recordFirstFrame();
                setLoading(false);
                setReconnecting(false);
              };
            }
          }
        }
      };

      // 연결 상태 변경 이벤트 처리
      pc.onconnectionstatechange = () => {
        if (!mountedRef.current) return;
        console.log('P2P 연결 상태:', pc.connectionState);
        
        if (pc.connectionState === 'connected') {
          console.log('P2P 연결 성공');
          performanceMonitorRef.current.recordConnectionEstablished();
          setLoading(false);
          setReconnecting(false);
        }
        else if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected' || pc.connectionState === 'closed') {
          console.error('P2P 연결 실패 또는 종료:', pc.connectionState);
          setReconnecting(true);
          
          // 서버 측 문제로 인한 연결 실패인 경우
          if (pc.connectionState === 'failed') {
            setErrorMessage('서버와의 P2P 연결 실패. 서버 상태를 확인하세요.');
            
            // Rate limit을 고려한 재연결 지연
            const timeSinceLastOffer = Date.now() - lastOfferTimeRef.current;
            const minDelay = Math.max(MIN_OFFER_INTERVAL - timeSinceLastOffer, 12000);
            
            reconnectTimeoutRef.current = setTimeout(() => {
              if (mountedRef.current) {
                console.log(`P2P 연결 실패. ${minDelay/1000}초 후 재연결 시도...`);
                initializeConnection();
              }
            }, minDelay);
          }
        }
      };

      // 오퍼 생성 및 전송
      try {
        const offer = await pc.createOffer({
          offerToReceiveVideo: true,
          offerToReceiveAudio: true
        });
        
        // Offer 생성 시간 기록
        performanceMonitorRef.current.recordOfferCreated();
        
        // 클라이언트 offer SDP 로깅 (디버깅용)
        console.log('클라이언트 생성 offer SDP 전송');
        
        await pc.setLocalDescription(offer);
        console.log('Local description set successfully');

        // 서버로 offer 전송
        const response = await sendOfferViaHttp(offer);
        
        if (response) {
          // Answer 수신 시간 기록
          performanceMonitorRef.current.recordAnswerReceived();
          
          // 세션 ID 저장
          setSessionId(response.sessionId);
          
          // 서버 answer SDP 로깅
          console.log('서버 answer SDP 수신');
          
          // 오류 처리 로직 추가
          try {
            // 수신한 SDP 정리 및 처리
            let cleanedSdp = response.sdp;
            try {
              cleanedSdp = validateSdp(response.sdp);
            } catch (sdpError) {
              console.warn('SDP 정리 실패, 원본 SDP 사용:', sdpError.message);
            }
            
            // 원격 SDP 설정
            const rtcSessionDescription = new RTCSessionDescription({
              type: 'answer',
              sdp: cleanedSdp
            });
            
            try {
              // RTCPeerConnection 상태 검사 - closed 상태에서는 setRemoteDescription 호출 금지
              if (!pc || pc.signalingState === 'closed') {
                console.warn('RTCPeerConnection이 closed 상태입니다. setRemoteDescription 생략');
                console.log('현재 PC 상태:', pc ? pc.signalingState : 'null');
                throw new Error('RTCPeerConnection closed - 재연결 필요');
              }
              
              console.log('setRemoteDescription 호출 전 PC 상태:', pc.signalingState);
              await pc.setRemoteDescription(rtcSessionDescription);
              console.log('원격 SDP 설정 완료, 새로운 PC 상태:', pc.signalingState);
              remoteDescriptionSetRef.current = true;
              
              // 서버가 제공한 ICE 후보 추가
              if (response.iceCandidates && response.iceCandidates.length > 0) {
                for (const candidateInfo of response.iceCandidates) {
                  try {
                    // ICE 후보 추가 전에도 연결 상태 확인
                    if (pc.signalingState === 'closed') {
                      console.warn('PC가 closed 상태가 되어 ICE 후보 추가 중단');
                      break;
                    }
                    
                    const candidate = new RTCIceCandidate(candidateInfo);
                    await pc.addIceCandidate(candidate);
                    console.log('서버에서 받은 ICE 후보 추가됨');
                  } catch (e) {
                    console.error('ICE 후보 추가 실패:', e);
                  }
                }
              }
            } catch (sdpError) {
              console.error('SDP 설정 오류:', sdpError.message);
              
              // RTCPeerConnection closed 오류에 대한 특별 처리
              if (sdpError.message.includes('RTCPeerConnection closed')) {
                console.warn('RTCPeerConnection 상태 오류로 인한 재연결 필요');
                setErrorMessage('연결이 종료됨 - 재연결 시도 중');
                throw new Error('RTCPeerConnection closed - 재연결 필요');
              }
              
              // 복구 시도: 특정 SDP 형식 오류 처리
              if (sdpError.message.includes('Failed to parse SessionDescription') || 
                  sdpError.message.includes('Failed to execute') ||
                  sdpError.message.includes('Invalid SDP')) {
                console.warn('SDP 파싱 오류 감지, 재연결 시도...');
                setErrorMessage('SDP 형식 오류 - 재연결 시도 중');
                throw new Error('SDP 형식 오류 - 재연결 필요');
              } else if (sdpError.message.includes('order of m-lines')) {
                setErrorMessage('미디어 라인 순서 오류 - 재연결 시도 중');
                throw new Error('SDP 미디어 라인 순서 오류 - 재연결 필요');
              } else {
                throw sdpError;
              }
            }
          } catch (sdpError) {
            console.error('SDP 설정 오류:', sdpError.message);
            throw sdpError;
          }
        } else {
          throw new Error('서버에서 응답을 받지 못했습니다.');
        }
      } catch (e) {
        console.error('Offer 생성 또는 전송 중 오류:', e);
        throw e;
      }

    } catch (err) {
      console.error('RTCPeerConnection 설정 중 오류:', err);
      setErrorMessage('미디어 연결 초기화 실패');
      setLoading(false);
      setReconnecting(true);
      onError && onError(err);

      // Rate limit을 고려한 재연결 지연
      const timeSinceLastOffer = Date.now() - lastOfferTimeRef.current;
      const minDelay = Math.max(MIN_OFFER_INTERVAL - timeSinceLastOffer, 5000);

      if (mountedRef.current) {
        reconnectTimeoutRef.current = setTimeout(() => {
          if (mountedRef.current) {
            console.log(`${minDelay/1000}초 후 재연결 시도...`);
            initializeConnection();
          }
        }, minDelay);
      }
    }
  }, [iceServers, onConnected, onError]);

  // 연결 초기화
  useEffect(() => {
    console.log('LiveStreamComponent useEffect 실행 - streamId:', streamId);
    
    mountedRef.current = true;
    attemptRef.current = 0;
    lastIceConnectedTimeRef.current = null;
    remoteDescriptionSetRef.current = false;
    iceCandidatesQueueRef.current = [];
    
    // 이전 연결 정리
    cleanupConnection();
    
    // 초기 연결 시작 - 약간 지연시켜 컴포넌트가 완전히 마운트되도록 함
    const initTimer = setTimeout(() => {
      if (mountedRef.current) {
        console.log('초기 연결 시작 예약됨');
        initializeConnection();
      }
    }, 500); // 지연 시간을 500ms로 증가
    
    // 정리 함수
    return () => {
      console.log('LiveStreamComponent cleanup 실행');
      mountedRef.current = false;
      
      // 초기화 타이머 정리
      if (initTimer) {
        clearTimeout(initTimer);
      }
      
      // 재연결 타이머 정리
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
      
      // 연결 정리
      cleanupConnection();
    };
  }, [streamId]); // 의존성을 streamId만으로 제한

  // 수동 새로고침 핸들러
  const handleRefresh = useCallback(() => {
    if (!mountedRef.current) return;
    
    console.log('수동 새로고침 시작');
    attemptRef.current = 0;
    setErrorMessage(null);
    
    // 이전 연결 정리
    cleanupConnection();
    
    // 약간 지연 후 재연결 시도
    setTimeout(() => {
      if (mountedRef.current) {
        initializeConnection();
      }
    }, 100);
  }, []); // 의존성 제거

  // 컴포넌트 생명주기 추적
  useEffect(() => {
    console.log('LiveStreamComponent 마운트됨 - streamId:', streamId);
    return () => {
      console.log('LiveStreamComponent 언마운트됨 - streamId:', streamId);
    };
  }, [streamId]);
  

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
      
      {rateLimitMessage && (
        <div className="absolute top-4 left-4 right-4 z-10">
          <div className="bg-orange-600 bg-opacity-90 text-white px-4 py-2 rounded-md shadow-md">
            <div className="flex items-center">
              <span className="text-lg mr-2">⏱️</span>
              <span className="text-sm">{rateLimitMessage}</span>
            </div>
          </div>
        </div>
      )}
      
      {reconnecting && !loading && (
        <div className="absolute inset-0 flex items-center justify-center bg-gray-900 bg-opacity-75">
          <div className="flex flex-col items-center">
            <div className="text-yellow-400 mb-2">⚠️</div>
            <p className="text-white text-sm">스트림 연결 문제 발생</p>
            {errorMessage && (
              <p className="text-red-300 text-xs mt-1 max-w-xs text-center">{errorMessage}</p>
            )}
            <button 
              className="mt-3 px-4 py-1 bg-blue-500 text-white rounded-md text-sm"
              onClick={handleRefresh}
            >
              새로고침
            </button>
          </div>
        </div>
      )}
      
      <video
        ref={videoRef}
        className="w-full h-full aspect-video"
        autoPlay
        playsInline
        muted
      />
    </div>
  );
};

export default LiveStreamComponent;
