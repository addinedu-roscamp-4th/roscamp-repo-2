import React, { createContext, useState, useContext, useEffect } from 'react';
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || '';

const AuthContext = createContext(null);

export const useAuth = () => useContext(AuthContext);

export const AuthProvider = ({ children }) => {
  const [currentUser, setCurrentUser] = useState(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Check if there's a token in local storage
    const token = localStorage.getItem('token');
    if (token) {
      validateToken(token);
    } else {
      setIsLoading(false);
    }
  }, []);

  const validateToken = async (token) => {
    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/validate`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`
        }
      });

      if (response.ok) {
        const userData = await response.json();
        setCurrentUser(userData);
        setIsAuthenticated(true);
      } else {
        // Token is invalid, clear it
        localStorage.removeItem('token');
        setIsAuthenticated(false);
      }
    } catch (error) {
      console.error('Token validation failed:', error);
      localStorage.removeItem('token');
      setIsAuthenticated(false);
    } finally {
      setIsLoading(false);
    }
  };

  const login = async (username, password) => {
    setIsLoading(true);
    try {
      console.log('Attempting login for user:', username);
      
      // For OAuth2 we need to use the correct form data format
      const formData = new URLSearchParams();
      formData.append('username', username);
      formData.append('password', password);
      
      // Print the URL for debugging
      const loginUrl = `${API_BASE_URL}/api/auth/login`;
      console.log('Login URL:', loginUrl);
      
      const response = await fetch(loginUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded'
        },
        body: formData
      });

      console.log('Login response status:', response.status);
      
      // Check for 4xx/5xx errors that need to be processed
      if (response.ok) {
        const data = await response.json();
        console.log('Login success, token received:', data);
        localStorage.setItem('token', data.access_token);
        
        // Fetch user data with the new token
        const userResponse = await fetch(`${API_BASE_URL}/api/users/me`, {
          headers: {
            'Authorization': `Bearer ${data.access_token}`
          }
        });
        
        console.log('User data response:', userResponse.status);
        
        if (userResponse.ok) {
          const userData = await userResponse.json();
          console.log('User data:', userData);
          setCurrentUser(userData);
          setIsAuthenticated(true);
          return { success: true };
        } else {
          console.warn('Could not fetch user data after login');
          // Still consider login successful even if we couldn't get user data
          setIsAuthenticated(true);
          return { success: true };
        }
      } else {
        let errorMessage = '로그인에 실패했습니다.';
        try {
          const errorData = await response.json();
          errorMessage = errorData.detail || errorMessage;
        } catch (e) {
          console.error('Could not parse error response:', e);
        }
        console.error('Login failed:', errorMessage);
        return { success: false, message: errorMessage };
      }
    } catch (error) {
      console.error('Login error:', error);
      return { success: false, message: '서버 오류가 발생했습니다.' };
    } finally {
      setIsLoading(false);
    }
  };

  const logout = () => {
    localStorage.removeItem('token');
    setCurrentUser(null);
    setIsAuthenticated(false);
  };

  // API 호출 함수
  const apiCall = async (url, method = 'GET', data = null) => {
    const token = localStorage.getItem('token');
    
    const headers = {
      'Content-Type': 'application/json'
    };
    
    if (token) {
      headers['Authorization'] = `Bearer ${token}`;
    }
    
    const config = {
      method,
      headers
    };
    
    if (data && (method === 'POST' || method === 'PUT' || method === 'PATCH')) {
      config.body = JSON.stringify(data);
    }
    
    try {
      const fullUrl = `${API_BASE_URL}${url}`;
      console.log(`API call ${method}:`, fullUrl);
      
      const response = await fetch(fullUrl, config);
      
      if (response.status === 401) {
        // Token expired or invalid
        logout();
        throw new Error('인증이 만료되었습니다. 다시 로그인해주세요.');
      }
      
      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || '요청 처리 중 오류가 발생했습니다.');
      }
      
      // For 204 No Content responses
      if (response.status === 204) {
        return null;
      }
      
      return await response.json();
    } catch (error) {
      console.error(`API call error (${url}):`, error);
      throw error;
    }
  };

  const value = {
    currentUser,
    isAuthenticated,
    isLoading,
    login,
    logout,
    apiCall
  };
  

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export default AuthContext; 