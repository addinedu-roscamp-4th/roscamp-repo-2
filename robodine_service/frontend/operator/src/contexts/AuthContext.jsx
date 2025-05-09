import React, { createContext, useState, useContext, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';

// API 기본 URL
const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

// 인증 컨텍스트 생성
const AuthContext = createContext(null);

// 인증 컨텍스트 프로바이더
export const AuthProvider = ({ children }) => {
  const [currentUser, setCurrentUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [isTokenValid, setIsTokenValid] = useState(false);
  const navigate = useNavigate();

  // 페이지 초기 로드시 자동 로그인 시도
  useEffect(() => {
    const checkAuth = async () => {
      const token = localStorage.getItem('token');
      if (token) {
        try {
          const user = await apiCall('/api/auth/me');
          setCurrentUser(user);
          setIsTokenValid(true);
          setLoading(false);
        } catch (err) {
          // 토큰이 유효하지 않으면 로컬 스토리지에서 제거
          console.error('Auth check failed:', err);
          localStorage.removeItem('token');
          setCurrentUser(null);
          setIsTokenValid(false);
          setLoading(false);
          navigate('/login');
        }
      } else {
        setLoading(false);
      }
    };

    checkAuth();
  }, [navigate]);

  // API 호출 유틸리티 함수
  const apiCall = async (endpoint, method = 'GET', data = null) => {
    try {
      const url = `${API_URL}${endpoint}`;
      console.log(`API 요청: ${method} ${url}`);
      
      const headers = {
        'Content-Type': 'application/json',
      };
      
      // 로그인한 상태면 토큰 추가
      const token = localStorage.getItem('token');
      if (token) {
        headers['Authorization'] = `Bearer ${token}`;
      }
      
      const config = {
        method,
        headers,
        credentials: 'include', // 쿠키 포함
      };
      
      if (data) {
        config.body = JSON.stringify(data);
      }
      
      const response = await fetch(url, config);
      
      // 응답 데이터 추출
      let responseData;
      const contentType = response.headers.get('content-type');
      if (contentType && contentType.includes('application/json')) {
        responseData = await response.json();
      } else {
        responseData = await response.text();
      }
      
      // 응답 상태 확인
      if (!response.ok) {
        // 오류 응답 생성
        const error = new Error(
          responseData.detail || 
          (typeof responseData === 'string' ? responseData : JSON.stringify(responseData)) || 
          `Status code: ${response.status}`
        );
        
        // 응답 내용 추가
        error.response = {
          status: response.status,
          statusText: response.statusText,
          data: responseData
        };
        
        // 401 Unauthorized 처리
        if (response.status === 401) {
          console.log('API 인증 오류, 로그아웃');
          // 토큰 삭제 및 사용자 상태 초기화
          localStorage.removeItem('token');
          setCurrentUser(null);
          setIsTokenValid(false);
          
          // 로그인 페이지로 리디렉션
          navigate('/login');
        }
        
        throw error;
      }
      
      return responseData;
    } catch (error) {
      console.error(`API call error (${endpoint}):`, error);
      throw error;
    }
  };

  // 로그인 처리
  const login = async (username, password) => {
    try {
      const formData = new URLSearchParams();
      formData.append('username', username);
      formData.append('password', password);
      
      const response = await fetch(`${API_URL}/api/auth/token`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: formData
      });

      const data = await response.json();
      
      if (!response.ok) {
        throw new Error(data.detail || '로그인에 실패했습니다');
      }
      
      // 토큰 저장 및 사용자 정보 설정
      localStorage.setItem('token', data.access_token);
      setIsTokenValid(true);
      
      try {
        const user = await apiCall('/api/auth/me');
        setCurrentUser(user);
        return user;
      } catch (userErr) {
        console.error('Failed to fetch user info after login:', userErr);
        // 사용자 정보 가져오기 실패해도 로그인은 성공으로 처리
        return { username };
      }
    } catch (err) {
      setError(err.message);
      throw err;
    }
  };

  // 회원가입 처리
  const signup = async (userData) => {
    try {
      const data = await apiCall('/api/auth/register', 'POST', userData);
      
      return data;
    } catch (err) {
      setError(err.message);
      throw err;
    }
  };

  // 로그아웃 처리
  const logout = () => {
    localStorage.removeItem('token');
    setCurrentUser(null);
    setIsTokenValid(false);
    navigate('/login');
  };

  // 프로필 정보 업데이트
  const updateProfile = async (userData) => {
    try {
      const data = await apiCall('/api/users/me', 'PUT', userData);
      
      setCurrentUser({ ...currentUser, ...data });
      return data;
    } catch (err) {
      setError(err.message);
      throw err;
    }
  };

  // 컨텍스트 값
  const value = {
    currentUser,
    loading,
    error,
    isTokenValid,
    isAuthenticated: isTokenValid && !!currentUser,
    login,
    signup,
    logout,
    updateProfile,
    apiCall,
    setError
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

// 훅으로 인증 컨텍스트 사용
export const useAuth = () => {
  return useContext(AuthContext);
}; 