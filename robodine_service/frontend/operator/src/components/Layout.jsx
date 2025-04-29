import React, { useState } from 'react';
import { Link, useLocation, useNavigate } from 'react-router-dom';
import { 
  Menu, X, Home, User, Settings, Package, 
  Database, Bell, LogOut, Monitor, AlertOctagon,
  Coffee, Users, Video, Server
} from 'react-feather';
import { useAuth } from '../contexts/AuthContext';

const Header = ({ toggleSidebar }) => {
  const [showUserMenu, setShowUserMenu] = useState(false);
  const [notifications, setNotifications] = useState([
    { id: 1, message: '로봇 #2 배터리 부족', type: 'warning', time: '10분 전' },
    { id: 2, message: '테이블 #5 주문 완료', type: 'info', time: '15분 전' }
  ]);
  const [showNotifications, setShowNotifications] = useState(false);
  const { currentUser, logout } = useAuth();
  const navigate = useNavigate();

  const handleLogout = () => {
    logout();
    navigate('/login');
  };

  const clearNotifications = () => {
    setNotifications([]);
    setShowNotifications(false);
  };

  return (
    <header className="bg-white shadow-sm fixed top-0 left-0 right-0 z-10">
      <div className="flex justify-between items-center px-4 py-2">
        <div className="flex items-center">
          <button 
            onClick={toggleSidebar}
            className="mr-2 p-2 rounded-md text-gray-500 hover:text-gray-600 focus:outline-none"
          >
            <Menu size={24} />
          </button>
          <Link to="/" className="text-xl font-bold text-gray-800 flex items-center">
            <Coffee className="text-blue-600 mr-2" />
            <span>로보다인</span>
            <span className="text-blue-600">서비스</span>
          </Link>
        </div>
        
        <div className="flex items-center space-x-3">
          {/* 현재 시간 */}
          <div className="hidden md:block text-sm text-gray-600">
            {new Date().toLocaleString('ko-KR', {
              weekday: 'long',
              year: 'numeric',
              month: 'long',
              day: 'numeric',
              hour: '2-digit',
              minute: '2-digit'
            })}
          </div>
          
          {/* 알림 */}
          <div className="relative">
            <button 
              onClick={() => setShowNotifications(!showNotifications)}
              className="p-2 rounded-full text-gray-500 hover:text-gray-700 hover:bg-gray-100 relative"
            >
              <Bell size={20} />
              {notifications.length > 0 && (
                <span className="absolute top-0 right-0 block h-4 w-4 rounded-full bg-red-500 text-white text-xs flex items-center justify-center">
                  {notifications.length}
                </span>
              )}
            </button>
            
            {showNotifications && (
              <div className="absolute right-0 mt-2 bg-white rounded-md shadow-lg w-72 z-20">
                <div className="py-2 px-4 border-b flex justify-between items-center">
                  <h3 className="font-medium text-gray-700">알림</h3>
                  <button 
                    onClick={clearNotifications}
                    className="text-xs text-blue-600 hover:text-blue-800"
                  >
                    모두 읽음 처리
                  </button>
                </div>
                <div className="max-h-80 overflow-y-auto">
                  {notifications.length > 0 ? (
                    <div>
                      {notifications.map(notification => (
                        <div key={notification.id} className="px-4 py-3 hover:bg-gray-50 border-b last:border-0">
                          <p className="text-sm text-gray-800">{notification.message}</p>
                          <p className="text-xs text-gray-500 mt-1">{notification.time}</p>
                        </div>
                      ))}
                    </div>
                  ) : (
                    <div className="px-4 py-6 text-center text-gray-500">
                      <Bell size={24} className="mx-auto mb-2 opacity-25" />
                      <p className="text-sm">알림이 없습니다</p>
                    </div>
                  )}
                </div>
              </div>
            )}
          </div>
          
          {/* 사용자 메뉴 */}
          <div className="relative">
            <button 
              onClick={() => setShowUserMenu(!showUserMenu)}
              className="flex items-center p-2 rounded-full hover:bg-gray-100"
            >
              <div className="w-8 h-8 bg-blue-500 rounded-full flex items-center justify-center text-white font-bold">
                {currentUser?.name?.charAt(0) || currentUser?.username?.charAt(0) || 'U'}
              </div>
              <span className="hidden md:block ml-2 text-sm text-gray-700">
                {currentUser?.name || currentUser?.username}
              </span>
            </button>
            
            {showUserMenu && (
              <div className="absolute right-0 mt-2 bg-white rounded-md shadow-lg w-48 z-20">
                <div className="py-2">
                  <Link 
                    to="/profile" 
                    className="block px-4 py-2 text-sm text-gray-700 hover:bg-gray-100 flex items-center"
                    onClick={() => setShowUserMenu(false)}
                  >
                    <User size={16} className="mr-2" />
                    프로필
                  </Link>
                  <Link 
                    to="/settings" 
                    className="block px-4 py-2 text-sm text-gray-700 hover:bg-gray-100 flex items-center"
                    onClick={() => setShowUserMenu(false)}
                  >
                    <Settings size={16} className="mr-2" />
                    설정
                  </Link>
                  <button 
                    onClick={handleLogout}
                    className="w-full text-left px-4 py-2 text-sm text-gray-700 hover:bg-gray-100 flex items-center"
                  >
                    <LogOut size={16} className="mr-2" />
                    로그아웃
                  </button>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </header>
  );
};

const Sidebar = ({ isOpen, closeSidebar }) => {
  const location = useLocation();
  
  const menuItems = [
    { path: '/', icon: <Home size={20} />, label: '대시보드' },
    { path: '/robots', icon: <Server size={20} />, label: '로봇 관리' },
    { path: '/customers', icon: <Users size={20} />, label: '고객·테이블 관리' },
    { path: '/orders', icon: <Package size={20} />, label: '주문 관리' },
    { path: '/inventory', icon: <Database size={20} />, label: '재고 관리' },
    { path: '/video-streams', icon: <Video size={20} />, label: '영상 스트리밍' },
    { path: '/emergencies', icon: <AlertOctagon size={20} />, label: '비상 상황 관리' },
    { path: '/settings', icon: <Settings size={20} />, label: '설정' },
  ];
  
  return (
    <>
      {/* 모바일 오버레이 */}
      {isOpen && (
        <div 
          className="fixed inset-0 bg-black bg-opacity-50 z-20 lg:hidden"
          onClick={closeSidebar}
        ></div>
      )}
      
      {/* 사이드바 */}
      <aside className={`fixed top-14 bottom-0 left-0 z-20 w-64 bg-gray-800 text-white transform transition-transform duration-200 ease-in-out ${
        isOpen ? 'translate-x-0' : '-translate-x-full lg:translate-x-0'
      }`}>
        <div className="p-4">
          <div className="flex justify-between items-center mb-6 lg:hidden">
            <h2 className="text-xl font-bold">메뉴</h2>
            <button 
              onClick={closeSidebar}
              className="p-2 rounded-md hover:bg-gray-700"
            >
              <X size={20} />
            </button>
          </div>
          
          <nav>
            <ul className="space-y-2">
              {menuItems.map(item => (
                <li key={item.path}>
                  <Link
                    to={item.path}
                    className={`flex items-center px-4 py-3 rounded-md transition-colors ${
                      location.pathname === item.path
                        ? 'bg-blue-600 text-white'
                        : 'text-gray-300 hover:bg-gray-700'
                    }`}
                    onClick={closeSidebar}
                  >
                    <span className="mr-3">{item.icon}</span>
                    <span>{item.label}</span>
                  </Link>
                </li>
              ))}
            </ul>
          </nav>
        </div>
        
        <div className="absolute bottom-0 left-0 right-0 p-4 border-t border-gray-700">
          <div className="flex items-center">
            <Monitor size={20} className="text-gray-400 mr-3" />
            <div>
              <p className="text-sm font-medium text-white">시스템 상태</p>
              <p className="text-xs text-green-400">모든 시스템 정상 작동 중</p>
            </div>
          </div>
        </div>
      </aside>
    </>
  );
};

const Layout = ({ children }) => {
  const [sidebarOpen, setSidebarOpen] = useState(false);
  
  const toggleSidebar = () => setSidebarOpen(!sidebarOpen);
  const closeSidebar = () => setSidebarOpen(false);
  
  return (
    <div className="min-h-screen bg-gray-100">
      <Header toggleSidebar={toggleSidebar} />
      <Sidebar isOpen={sidebarOpen} closeSidebar={closeSidebar} />
      
      <main className="pt-14 lg:pl-64 transition-all duration-200">
        {children}
      </main>
    </div>
  );
};

export default Layout; 