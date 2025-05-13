import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import { useWebSockets } from './WebSocketContext';
import { useAuth } from './AuthContext';

// 기본 시스템 상태 정의
const initialSystemStatus = {
  overall: 'checking',
  services: {
    database: 'checking',
    backend: 'checking',
    robots: 'checking',
    inventory: 'checking'
  },
  lastChecked: null,
  errors: []
};

// 기본값을 가진 HealthCheckContext 생성
const HealthCheckContext = createContext({
  systemStatus: initialSystemStatus,
  performHealthCheck: () => Promise.resolve()
});

// 헬스체크 컨텍스트 프로바이더
export const HealthCheckProvider = ({ children }) => {
  const [systemStatus, setSystemStatus] = useState(initialSystemStatus);
  
  // 웹소켓 컨텍스트에서 데이터 및 연결 상태 가져오기
  const { data: wsData, errors: wsErrors, connected } = useWebSockets();
  const { apiCall } = useAuth();
  
  // 디버깅용 로그 함수
  const logDebug = (...args) => {
    // console.log('[헬스체크]', ...args);
  };

  // 웹소켓 상태 변경 로깅
  useEffect(() => {
    // logDebug("웹소켓 연결 상태:", connected);
  }, [connected]);

  // 헬스체크 수행 함수를 useCallback으로 메모이제이션
  const performHealthCheck = useCallback(async () => {
    logDebug("헬스체크 시작");
    
    try {
      // API 호출 시도
      const healthData = await apiCall('/api/health', 'GET');
      // logDebug("헬스체크 API 응답:", healthData); 
      
      // 웹소켓 연결 상태 확인
      const wsHealth = connected && 
                       connected.robots === true && 
                       connected.systemlogs === true && 
                       connected.status === true;
      
      // logDebug("웹소켓 연결 상태:", { connected, wsHealth });
      
      // 서비스별 상태 업데이트
      const servicesStatus = {
        database: healthData?.database || 'unhealthy',
        backend: healthData?.status || 'unhealthy',
        robots: wsHealth ? 'healthy' : 'unhealthy',
      };
      
      // logDebug("서비스 상태:", servicesStatus);
  
      const errors = [];
      if (servicesStatus.database !== 'healthy') errors.push('데이터베이스 연결 문제가 발생했습니다.');
      if (servicesStatus.backend !== 'healthy') errors.push('백엔드 API 서비스에 문제가 발생했습니다.');
      if (servicesStatus.robots !== 'healthy') errors.push('웹소켓 연결 서비스에 문제가 발생했습니다.');
  
      const allHealthy = Object.values(servicesStatus).every(status => status === 'healthy');
      
      setSystemStatus({
        overall: allHealthy ? 'healthy' : 'unhealthy',
        services: servicesStatus,
        lastChecked: new Date(),
        errors
      });
    } catch (err) {
      // logDebug('헬스체크 수행 중 오류 발생:', err);
      
      // 웹소켓 연결 상태 확인 (에러 상황에서도)
      const robotsConnected = connected && connected.robots === true;
      
      setSystemStatus({
        overall: 'unhealthy',
        services: {
          database: 'unknown',
          backend: 'unhealthy',
          robots: robotsConnected ? 'healthy' : 'unhealthy',
          inventory: 'unknown'
        },
        lastChecked: new Date(),
        errors: ['헬스체크 API 호출 중 오류가 발생했습니다: ' + (err.message || '알 수 없는 오류')]
      });
    }
  }, [apiCall, connected]); // 의존성 배열에 apiCall과 connected 추가
  
  // 초기 및 주기적 헬스체크 설정
  useEffect(() => {
    // logDebug("헬스체크 타이머 설정");
    
    // 초기 헬스체크는 약간의 지연 후 수행
    const initialCheckTimer = setTimeout(() => {
      // logDebug("초기 헬스체크 실행");
      performHealthCheck();
    }, 2000); // 2초로 약간 늘림
    
    // 정기적 헬스체크 설정 (5분마다)
    const intervalId = setInterval(() => {
      // logDebug("주기적 헬스체크 실행");
      performHealthCheck();
    }, 5 * 60 * 1000);
    
    return () => {
      // logDebug("헬스체크 타이머 정리");
      clearTimeout(initialCheckTimer);
      clearInterval(intervalId);
    };
  }, [performHealthCheck]); // performHealthCheck 의존성 추가
  
  // 웹소켓 연결 상태 변경 시 헬스체크 다시 수행
  useEffect(() => {
    // connected 객체가 존재하고 연결 상태가 변경되었을 때만 다시 체크
    if (connected && 
        (connected.robots !== undefined || 
         connected.systemlogs !== undefined || 
         connected.status !== undefined)) {
      // logDebug("웹소켓 상태 변경 감지:", connected);
      performHealthCheck();
    }
  }, [connected, performHealthCheck]); // performHealthCheck 의존성 추가

  return (
    <HealthCheckContext.Provider value={{
      systemStatus,
      performHealthCheck
    }}>
      {children}
    </HealthCheckContext.Provider>
  );
};

// 헬스체크 컨텍스트 사용 훅
export const useHealthCheck = () => {
  return useContext(HealthCheckContext);
};

export default HealthCheckProvider; 