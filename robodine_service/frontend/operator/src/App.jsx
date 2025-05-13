// src/App.jsx
import React from 'react';
import { Routes, Route, Navigate } from 'react-router-dom';

// Contexts
import { AuthProvider, useAuth } from './contexts/AuthContext';
import WebSocketProvider from './contexts/WebSocketContext';
import { HealthCheckProvider } from './contexts/HealthCheckContext';
import { NotificationsProvider } from './contexts/NotificationsContext';

// Components
import FloatingChat from './components/FloatingChat';

// Pages
import DashboardPage from './pages/DashboardPage';
import LoginPage from './pages/LoginPage';
import OrdersPage from './pages/OrdersPage';
import CustomerPage from './pages/CustomerPage';
import StatsPage from './pages/StatsPage';
import SettingsPage from './pages/SettingsPage';
import RobotAdminPage from './pages/RobotAdminPage';
import VideoStreamPage from './pages/VideoStreamPage';
import InventoryPage from './pages/InventoryPage';

// Protected Route Component
const ProtectedRoute = ({ children }) => {
  const { isAuthenticated, isLoading } = useAuth();
  if (isLoading) {
    return (
      <div className="flex items-center justify-center h-screen">
        <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
      </div>
    );
  }
  if (!isAuthenticated) {
    return <Navigate to="/login" />;
  }
  return children;
};

function App() {
  return (
    <AuthProvider>
      <WebSocketProvider>
        <HealthCheckProvider>
          <NotificationsProvider>
            <div className="App">
              <Routes>
                {/* Auth Route */}
                <Route path="/login" element={<LoginPage />} />

                {/* Protected Routes */}
                <Route
                  path="/"
                  element={
                    <ProtectedRoute>
                      <DashboardPage />
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/orders"
                  element={
                    <ProtectedRoute>
                      <OrdersPage />
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/customers"
                  element={
                    <ProtectedRoute>
                      <CustomerPage />
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/stats"
                  element={
                    <ProtectedRoute>
                      <StatsPage />
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/settings"
                  element={
                    <ProtectedRoute>
                      <SettingsPage />
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/robots"
                  element={
                    <ProtectedRoute>
                      <RobotAdminPage />
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/video-stream"
                  element={
                    <ProtectedRoute>
                      <VideoStreamPage />
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/inventory"
                  element={
                    <ProtectedRoute>
                      <InventoryPage />
                    </ProtectedRoute>
                  }
                />

                {/* Redirect unknown paths to dashboard */}
                <Route path="*" element={<Navigate to="/" />} />
              </Routes>
              
              {/* Floating Chat Component - 로그인 페이지가 아닐 때만 표시 */}
              <AuthComponent>
                <FloatingChat />
              </AuthComponent>
            </div>
          </NotificationsProvider>
        </HealthCheckProvider>
      </WebSocketProvider>
    </AuthProvider>
  );
}

// 인증 확인 컴포넌트 - 로그인 상태일 때만 자식 컴포넌트를 렌더링
const AuthComponent = ({ children }) => {
  const { isAuthenticated, isLoading } = useAuth();
  
  if (isLoading) return null;
  if (!isAuthenticated) return null;
  
  return children;
};

export default App;
