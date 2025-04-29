import React from 'react';
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom';
import { AuthProvider, useAuth } from './contexts/AuthContext';

// 페이지 컴포넌트 import
import LoginPage from './pages/LoginPage';
import SignupPage from './pages/SignupPage';
import DashboardPage from './pages/DashboardPage';
import RobotAdminPage from './pages/RobotAdminPage';
import CustomerPage from './pages/CustomerPage';
import VideoStreamPage from './pages/VideoStreamPage';
import EmergencyPage from './pages/EmergencyPage';
import SettingsPage from './pages/SettingsPage';
import OrdersPage from './pages/OrdersPage';
import InventoryPage from './pages/InventoryPage';

// 인증 확인 HOC
const PrivateRoute = ({ children }) => {
  const { currentUser, loading, isTokenValid } = useAuth();
  
  // 로딩 중일 때는 아무것도 표시하지 않음
  if (loading) {
    return <div className="flex items-center justify-center h-screen">
      <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
    </div>;
  }
  
  return isTokenValid ? children : <Navigate to="/login" />;
};

// 인증 래퍼 컴포넌트
const AuthWrapper = () => {
  return (
    <Router>
      <Routes>
        {/* 인증 페이지 */}
        <Route path="/login" element={<LoginPage />} />
        <Route path="/signup" element={<SignupPage />} />
        
        {/* 메인 대시보드 */}
        <Route 
          path="/" 
          element={
            <PrivateRoute>
              <DashboardPage />
            </PrivateRoute>
          } 
        />
        <Route 
          path="/dashboard" 
          element={
            <PrivateRoute>
              <DashboardPage />
            </PrivateRoute>
          } 
        />
        
        {/* 로봇 관리 */}
        <Route 
          path="/robots" 
          element={
            <PrivateRoute>
              <RobotAdminPage />
            </PrivateRoute>
          } 
        />
        
        {/* 고객/테이블 관리 */}
        <Route 
          path="/customers" 
          element={
            <PrivateRoute>
              <CustomerPage />
            </PrivateRoute>
          } 
        />
        
        {/* 영상 스트리밍 */}
        <Route 
          path="/video-streams" 
          element={
            <PrivateRoute>
              <VideoStreamPage />
            </PrivateRoute>
          } 
        />
        
        {/* 비상 상황 관리 */}
        <Route 
          path="/emergencies" 
          element={
            <PrivateRoute>
              <EmergencyPage />
            </PrivateRoute>
          } 
        />
        
        {/* 주문/재고 관리 */}
        <Route 
          path="/orders" 
          element={
            <PrivateRoute>
              <OrdersPage />
            </PrivateRoute>
          } 
        />
        <Route 
          path="/inventory" 
          element={
            <PrivateRoute>
              <InventoryPage />
            </PrivateRoute>
          } 
        />
        
        {/* 설정 */}
        <Route 
          path="/settings" 
          element={
            <PrivateRoute>
              <SettingsPage />
            </PrivateRoute>
          } 
        />
        
        {/* 404 - 홈으로 리다이렉트 */}
        <Route path="*" element={<Navigate to="/" />} />
      </Routes>
    </Router>
  );
};

const App = () => {
  return (
    <AuthProvider>
      <AuthWrapper />
    </AuthProvider>
  );
};

export default App; 