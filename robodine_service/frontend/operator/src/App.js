import React from 'react';
import { Routes, Route, Navigate } from 'react-router-dom';

// Pages
import DashboardPage from './pages/DashboardPage';
import LoginPage from './pages/LoginPage';

// Auth Context
import { AuthProvider, useAuth } from './contexts/AuthContext';

// Import more pages here as needed
// Example:
// import VideoStreamPage from './pages/VideoStreamPage';
// import RobotAdminPage from './pages/RobotAdminPage';
// import CustomerPage from './pages/CustomerPage';
// import StatsPage from './pages/StatsPage';
// import SystemPage from './pages/SystemPage';
// import SettingsPage from './pages/SettingsPage';

// Base API URL
// Use empty string for relative URLs (same server) or specify the full base URL for different server
export const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || '';

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
      <div className="App">
        <Routes>
          {/* Auth Routes */}
          <Route path="/login" element={<LoginPage />} />
          
          {/* Protected Routes */}
          <Route path="/" element={
            <ProtectedRoute>
              <DashboardPage />
            </ProtectedRoute>
          } />
          
          {/* Redirect any unknown routes to dashboard */}
          <Route path="*" element={<Navigate to="/" />} />
        </Routes>
      </div>
    </AuthProvider>
  );
}

export default App; 