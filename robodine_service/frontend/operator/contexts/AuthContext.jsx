import React, { createContext, useState, useContext, useEffect } from 'react';

const AuthContext = createContext();

export const useAuth = () => useContext(AuthContext);

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    // Check for existing token and validate it
    const checkAuth = async () => {
      const token = localStorage.getItem('auth_token');
      
      if (!token) {
        setLoading(false);
        return;
      }

      try {
        // Validate token with backend
        const response = await fetch('/api/auth/validate', {
          headers: {
            'Authorization': `Bearer ${token}`
          }
        });

        if (response.ok) {
          const userData = await response.json();
          setUser(userData);
        } else {
          // Invalid token, clear storage
          localStorage.removeItem('auth_token');
        }
      } catch (error) {
        console.error('Authentication error:', error);
        setError('Failed to authenticate');
      } finally {
        setLoading(false);
      }
    };

    checkAuth();
  }, []);

  const login = async (username, password) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/auth/login', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: new URLSearchParams({
          username,
          password,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        localStorage.setItem('auth_token', data.access_token);
        
        // Get user data
        const userResponse = await fetch('/api/users/me', {
          headers: {
            'Authorization': `Bearer ${data.access_token}`
          }
        });

        if (userResponse.ok) {
          const userData = await userResponse.json();
          setUser(userData);
        }
        
        return { success: true };
      } else {
        const errorData = await response.json();
        setError(errorData.detail || '로그인에 실패했습니다');
        return { success: false, error: errorData.detail };
      }
    } catch (error) {
      console.error('Login error:', error);
      setError('로그인 중 오류가 발생했습니다');
      return { success: false, error: '로그인 중 오류가 발생했습니다' };
    } finally {
      setLoading(false);
    }
  };

  const register = async (username, password, name) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/users', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          username,
          password,
          name,
          role: 'ADMIN', // Default to admin for the admin dashboard
        }),
      });

      if (response.ok) {
        return { success: true };
      } else {
        const errorData = await response.json();
        setError(errorData.detail || '회원가입에 실패했습니다');
        return { success: false, error: errorData.detail };
      }
    } catch (error) {
      console.error('Registration error:', error);
      setError('회원가입 중 오류가 발생했습니다');
      return { success: false, error: '회원가입 중 오류가 발생했습니다' };
    } finally {
      setLoading(false);
    }
  };

  const logout = () => {
    localStorage.removeItem('auth_token');
    setUser(null);
  };

  const value = {
    user,
    login,
    register,
    logout,
    loading,
    error,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export default AuthContext; 