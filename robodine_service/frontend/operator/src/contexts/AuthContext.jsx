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
        } catch (err) {
          // 토큰이 유효하지 않으면 로컬 스토리지에서 제거
          console.error('Auth check failed:', err);
          localStorage.removeItem('token');
          setCurrentUser(null);
          setIsTokenValid(false);
          navigate('/login');
        }
      }
      setLoading(false);
    };

    checkAuth();
  }, [navigate]);

  // API 호출 유틸리티 함수
  const apiCall = async (endpoint, options = {}) => {
    setError(null);
    const token = localStorage.getItem('token');
    const url = `${API_URL}${endpoint}`;

    // 요청 헤더 설정
    const headers = {
      'Content-Type': 'application/json',
      ...(token && { Authorization: `Bearer ${token}` }),
      ...(options.headers || {})
    };

    try {
      const response = await fetch(url, {
        ...options,
        headers
      });

      // 응답이 JSON이 아닌 경우 처리
      const contentType = response.headers.get('content-type');
      if (contentType && contentType.indexOf('application/json') !== -1) {
        const data = await response.json();

        // 401/403 에러 처리 - 인증 실패
        if (!response.ok) {
          if (response.status === 401 || response.status === 403) {
            localStorage.removeItem('token');
            setCurrentUser(null);
            setIsTokenValid(false);
            navigate('/login');
          }
          throw new Error(data.detail || data.message || '요청 처리 중 오류가 발생했습니다');
        }

        return data;
      } else {
        // 응답이 JSON이 아닌 경우
        if (!response.ok) {
          if (response.status === 401 || response.status === 403) {
            localStorage.removeItem('token');
            setCurrentUser(null);
            setIsTokenValid(false);
            navigate('/login');
          }
          throw new Error('요청 처리 중 오류가 발생했습니다');
        }
        return await response.text();
      }
    } catch (err) {
      console.error(`API Error (${endpoint}):`, err.message);
      
      // 404 오류는 리소스가 없음을 나타내므로 특별히 처리
      if (err.message.includes('404') || err.message.includes('Not Found')) {
        setError(`API 엔드포인트를 찾을 수 없습니다: ${endpoint}`);
      } else {
        setError(err.message);
      }
      throw err;
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
      const data = await apiCall('/api/auth/register', {
        method: 'POST',
        body: JSON.stringify(userData)
      });
      
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
      const data = await apiCall('/api/users/me', {
        method: 'PUT',
        body: JSON.stringify(userData)
      });
      
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
    login,
    signup,
    logout,
    updateProfile,
    apiCall,
    setError
  };

  return (
    <AuthContext.Provider value={value}>
      {!loading && children}
    </AuthContext.Provider>
  );
};

// 훅으로 인증 컨텍스트 사용
export const useAuth = () => {
  return useContext(AuthContext);
}; 