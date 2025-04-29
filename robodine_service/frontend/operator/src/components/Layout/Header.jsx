import React, { useState, useEffect } from 'react';
import { format } from 'date-fns';
import { Bell, User, Menu, LogOut } from 'react-feather';
import { useAuth } from '../../contexts/AuthContext';
import { useNavigate } from 'react-router-dom';

const Header = ({ toggleSidebar }) => {
  const [currentTime, setCurrentTime] = useState(new Date());
  const [isNotificationsOpen, setIsNotificationsOpen] = useState(false);
  const [isUserMenuOpen, setIsUserMenuOpen] = useState(false);
  const [notifications, setNotifications] = useState([]);
  
  const { currentUser, logout, apiCall } = useAuth();
  const navigate = useNavigate();

  useEffect(() => {
    const timer = setInterval(() => setCurrentTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  useEffect(() => {
    // Close dropdowns when clicking outside
    const handleClickOutside = () => {
      setIsNotificationsOpen(false);
      setIsUserMenuOpen(false);
    };
    
    document.addEventListener('click', handleClickOutside);
    
    return () => {
      document.removeEventListener('click', handleClickOutside);
    };
  }, []);

  useEffect(() => {
    // Fetch notifications for the current user
    const fetchNotifications = async () => {
      try {
        // Only try to fetch if we have a user
        if (currentUser?.id) {
          const data = await apiCall(`/api/users/${currentUser.id}/notifications`).catch(() => []);
          setNotifications(Array.isArray(data) ? data : []);
        }
      } catch (error) {
        console.error('Failed to fetch notifications:', error);
        setNotifications([]);
      }
    };

    if (currentUser?.id) {
      fetchNotifications();
    }
  }, [currentUser, apiCall]);

  const handleLogout = () => {
    logout();
    navigate('/login');
  };

  const handleToggleNotifications = (e) => {
    e.stopPropagation();
    setIsNotificationsOpen(!isNotificationsOpen);
    setIsUserMenuOpen(false);
  };

  const handleToggleUserMenu = (e) => {
    e.stopPropagation();
    setIsUserMenuOpen(!isUserMenuOpen);
    setIsNotificationsOpen(false);
  };

  return (
    <header className="flex items-center justify-between px-6 py-4 bg-white border-b-1 shadow">
      <div className="flex items-center">
        <button 
          onClick={toggleSidebar}
          className="text-gray-500 focus:outline-none mr-4"
        >
          <Menu size={24} />
        </button>
        <div className="text-2xl font-bold text-gray-800">RoboDine Admin</div>
      </div>

      <div className="flex items-center space-x-6">
        <div className="text-gray-600">
          {format(currentTime, 'yyyy-MM-dd HH:mm:ss')}
        </div>

        <div className="relative">
          <button 
            onClick={handleToggleNotifications}
            className="text-gray-500 focus:outline-none relative"
          >
            <Bell size={24} />
            {notifications.length > 0 && (
              <span className="absolute -top-1 -right-1 bg-red-500 text-white rounded-full w-5 h-5 flex items-center justify-center text-xs">
                {notifications.length}
              </span>
            )}
          </button>

          {isNotificationsOpen && (
            <div className="absolute right-0 mt-2 w-80 bg-white rounded-md shadow-lg py-1 z-10" onClick={e => e.stopPropagation()}>
              <div className="px-4 py-2 text-sm font-medium text-gray-700 border-b">
                알림
              </div>
              <div className="max-h-60 overflow-y-auto">
                {notifications.length > 0 ? (
                  notifications.map((notification) => (
                    <div key={notification.id} className="px-4 py-2 hover:bg-gray-100">
                      <div className="font-medium text-gray-800">{notification.type}</div>
                      <div className="text-sm text-gray-600">{notification.message}</div>
                      <div className="text-xs text-gray-500">
                        {format(new Date(notification.created_at), 'yyyy-MM-dd HH:mm')}
                      </div>
                    </div>
                  ))
                ) : (
                  <div className="px-4 py-2 text-sm text-gray-500">알림 없음</div>
                )}
              </div>
            </div>
          )}
        </div>

        <div className="relative">
          <button 
            onClick={handleToggleUserMenu}
            className="flex items-center text-gray-700 focus:outline-none"
          >
            <div className="h-8 w-8 rounded-full bg-gray-300 flex items-center justify-center">
              <User size={18} />
            </div>
            <span className="ml-2">{currentUser?.name || currentUser?.username || '사용자'}</span>
          </button>

          {isUserMenuOpen && (
            <div className="absolute right-0 mt-2 w-48 bg-white rounded-md shadow-lg py-1 z-10" onClick={e => e.stopPropagation()}>
              <div className="px-4 py-2 text-sm text-gray-700 border-b">
                로그인: <span className="font-medium">{currentUser?.username || '사용자'}</span>
              </div>
              <button 
                onClick={handleLogout}
                className="block w-full text-left px-4 py-2 text-sm text-gray-700 hover:bg-gray-100"
              >
                <div className="flex items-center">
                  <LogOut size={16} className="mr-2" />
                  로그아웃
                </div>
              </button>
            </div>
          )}
        </div>
      </div>
    </header>
  );
};

export default Header; 