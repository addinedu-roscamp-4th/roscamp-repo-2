import React, { useState, useEffect } from 'react';
import { Link, useLocation, useNavigate } from 'react-router-dom';
import { 
  Menu, X, Home, Settings, Package, 
  Database, Bell, LogOut, Monitor, AlertOctagon,
  Coffee, Users, Video, Server, CloudLightning, AlertTriangle, ShoppingCart, RefreshCw,
  CheckCircle, XCircle, Info, HelpCircle
} from 'react-feather';
import { useAuth } from '../contexts/AuthContext';
import { useNotifications } from '../contexts/NotificationsContext';
import { useHealthCheck } from '../contexts/HealthCheckContext';

// 시스템 상태 모달 컴포넌트
const SystemStatusModal = ({ isOpen, onClose, systemStatus, onRefresh }) => {
  if (!isOpen) return null;

  // 상태에 따른 아이콘, 색상 반환 함수
  const getStatusIcon = (status) => {
    switch(status) {
      case 'healthy':
        return <CheckCircle className="text-green-500" size={18} />;
      case 'unhealthy':
        return <XCircle className="text-red-500" size={18} />;
      case 'checking':
        return <HelpCircle className="text-yellow-500" size={18} />;
      default:
        return <Info className="text-gray-500" size={18} />;
    }
  };

  // 디버깅 로그
  console.log('[시스템 상태 모달] 현재 시스템 상태:', systemStatus);

  return (
    <div className="fixed inset-0 z-50 overflow-y-auto">
      <div className="flex items-center justify-center min-h-screen pt-4 px-4 pb-20 text-center sm:block sm:p-0">
        <div className="fixed inset-0 transition-opacity" onClick={onClose}>
          <div className="absolute inset-0 bg-black opacity-50"></div>
        </div>

        <span className="hidden sm:inline-block sm:align-middle sm:h-screen"></span>&#8203;
        
        <div className="inline-block align-bottom bg-white rounded-lg text-left overflow-hidden shadow-xl transform transition-all sm:my-8 sm:align-middle sm:max-w-lg sm:w-full">
          <div className="bg-white px-4 pt-5 pb-4 sm:p-6 sm:pb-4">
            <div className="sm:flex sm:items-start">
              <div className="mt-3 text-center sm:mt-0 sm:ml-4 sm:text-left w-full">
                <h3 className="text-lg leading-6 font-medium text-gray-900 flex items-center">
                  <Monitor className="mr-2" size={20} />
                  시스템 상태 상세 정보
                </h3>
                <div className="mt-4">
                  <div className="bg-gray-50 p-4 rounded-lg mb-4">
                    <div className="flex items-center mb-2">
                      {getStatusIcon(systemStatus.overall)}
                      <span className="ml-2 font-semibold">전체 시스템 상태:</span>
                      <span className={`ml-2 ${
                        systemStatus.overall === 'healthy' ? 'text-green-600' :
                        systemStatus.overall === 'unhealthy' ? 'text-red-600' : 'text-yellow-600'
                      }`}>
                        {systemStatus.overall === 'healthy' ? '정상' : 
                         systemStatus.overall === 'unhealthy' ? '오류' : '확인 중'}
                      </span>
                    </div>
                    <p className="text-sm text-gray-500">
                      마지막 확인: {systemStatus.lastChecked ? new Date(systemStatus.lastChecked).toLocaleString('ko-KR') : '확인된 적 없음'}
                    </p>
                  </div>
                  
                  <h4 className="font-medium mb-2">서비스별 상태</h4>
                  <div className="space-y-2">
                    <div className="flex items-center justify-between p-2 border-b">
                      <div className="flex items-center">
                        <Database size={16} className="mr-2" />
                        <span>데이터베이스</span>
                      </div>
                      <div className="flex items-center">
                        {getStatusIcon(systemStatus.services.database)}
                        <span className={`ml-1 ${
                          systemStatus.services.database === 'healthy' ? 'text-green-600' :
                          systemStatus.services.database === 'unhealthy' ? 'text-red-600' : 'text-yellow-600'
                        }`}>
                          {systemStatus.services.database === 'healthy' ? '정상' : 
                           systemStatus.services.database === 'unhealthy' ? '오류' : '확인 중'}
                        </span>
                      </div>
                    </div>
                    <div className="flex items-center justify-between p-2 border-b">
                      <div className="flex items-center">
                        <Server size={16} className="mr-2" />
                        <span>백엔드 API</span>
                      </div>
                      <div className="flex items-center">
                        {getStatusIcon(systemStatus.services.backend)}
                        <span className={`ml-1 ${
                          systemStatus.services.backend === 'healthy' ? 'text-green-600' :
                          systemStatus.services.backend === 'unhealthy' ? 'text-red-600' : 'text-yellow-600'
                        }`}>
                          {systemStatus.services.backend === 'healthy' ? '정상' : 
                           systemStatus.services.backend === 'unhealthy' ? '오류' : '확인 중'}
                        </span>
                      </div>
                    </div>
                    <div className="flex items-center justify-between p-2 border-b">
                      <div className="flex items-center">
                        <CloudLightning size={16} className="mr-2" />
                        <span>웹소켓 연결</span>
                      </div>
                      <div className="flex items-center">
                        {getStatusIcon(systemStatus.services.robots)}
                        <span className={`ml-1 ${
                          systemStatus.services.robots === 'healthy' ? 'text-green-600' :
                          systemStatus.services.robots === 'unhealthy' ? 'text-red-600' : 'text-yellow-600'
                        }`}>
                          {systemStatus.services.robots === 'healthy' ? '정상' : 
                           systemStatus.services.robots === 'unhealthy' ? '오류' : '확인 중'}
                        </span>
                      </div>
                    </div>
                  </div>
                  
                  {systemStatus.errors && systemStatus.errors.length > 0 && (
                    <div className="mt-4">
                      <h4 className="font-medium mb-2 flex items-center">
                        <AlertTriangle size={16} className="text-red-500 mr-2" />
                        오류 메시지
                      </h4>
                      <div className="bg-red-50 border border-red-100 rounded-md p-3">
                        <ul className="list-disc pl-5 space-y-1">
                          {systemStatus.errors.map((error, index) => (
                            <li key={index} className="text-red-700 text-sm">{error}</li>
                          ))}
                        </ul>
                      </div>
                    </div>
                  )}
                </div>
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-4 py-3 sm:px-6 sm:flex sm:flex-row-reverse">
            <button
              type="button"
              onClick={onRefresh}
              className="w-full inline-flex justify-center rounded-md border border-transparent shadow-sm px-4 py-2 bg-blue-600 text-base font-medium text-white hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 sm:ml-3 sm:w-auto sm:text-sm"
            >
              <RefreshCw size={16} className="mr-2" />
              상태 새로고침
            </button>
            <button
              type="button"
              onClick={onClose}
              className="mt-3 w-full inline-flex justify-center rounded-md border border-gray-300 shadow-sm px-4 py-2 bg-white text-base font-medium text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500 sm:mt-0 sm:ml-3 sm:w-auto sm:text-sm"
            >
              닫기
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

const Header = ({ toggleSidebar }) => {
  const [showUserMenu, setShowUserMenu] = useState(false);
  const [showNotifications, setShowNotifications] = useState(false);
  const { currentUser, logout } = useAuth();
  const notificationsContext = useNotifications();
  const navigate = useNavigate();
  
  // 방어적 프로그래밍: notificationsContext가 없는 경우를 처리
  const notifications = notificationsContext?.notifications || [];
  const unreadCount = notificationsContext?.unreadCount || 0;
  const markAsRead = notificationsContext?.markAsRead || (() => {});
  const markAllAsRead = notificationsContext?.markAllAsRead || (() => {});

  // 알림 상태 로깅
  useEffect(() => {
    // console.log('[헤더] 알림 상태:', { 총알림수: notifications.length, 안읽은알림: unreadCount });
  }, [notifications.length, unreadCount]);

  const handleLogout = () => {
    logout();
    navigate('/login');
  };

  const clearNotifications = () => {
    // console.log('[헤더] 모든 알림 읽음 처리');
    markAllAsRead();
    setShowNotifications(false);
  };

  // 알림 클릭 처리
  const handleNotificationClick = (notification) => {
    // console.log('[헤더] 알림 클릭:', notification);
    markAsRead(notification.id);
    // 알림 타입에 따라 다른 페이지로 이동
    if (notification.source === 'systemlog') {
      navigate('/');  // 시스템 로그가 표시되는 대시보드로 이동
    }
  };

  // 알림 아이콘 가져오기
  const getNotificationIcon = (type) => {
    switch (type) {
      case 'INFO':
        return <Server className="text-blue-500" size={20} />;
      case 'WARNING':
        return <AlertTriangle className="text-yellow-500" size={20} />;
      case 'ERROR':
        return <AlertOctagon className="text-red-500" size={20} />;
      case 'DEBUG':
        return <Database className="text-green-500" size={20} />;
      default:
        return <Bell className="text-gray-500" size={20} />;
    }
  };

  // 알림 시간 포맷팅
  const formatTimestamp = (timestamp) => {
    if (!timestamp) return '';
    
    try {
      const now = new Date();
      const date = new Date(timestamp);
      
      if (isNaN(date.getTime())) return '';
      
      const diffMs = now - date;
      const diffMin = Math.floor(diffMs / (1000 * 60));
      
      // 1시간 미만이면 분 단위로 표시
      if (diffMin < 60) {
        if (diffMin < 1) return '방금 전';
        return `${diffMin}분 전`;
      }
      
      // 24시간 미만이면 시간 단위로 표시
      const diffHours = Math.floor(diffMin / 60);
      if (diffHours < 24) {
        return `${diffHours}시간 전`;
      }
      
      // 그 외에는 날짜와 시간 표시
      return date.toLocaleString('ko-KR', {
        month: 'short',
        day: 'numeric',
        hour: '2-digit',
        minute: '2-digit'
      });
    } catch (err) {
      console.error('시간 포맷팅 오류:', err);
      return '';
    }
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
              {unreadCount > 0 && (
                <span className="absolute top-0 right-0 block h-4 w-4 rounded-full bg-red-500 text-white text-xs flex items-center justify-center">
                  {unreadCount}
                </span>
              )}
            </button>
            
            {showNotifications && (
              <div className="absolute right-0 mt-2 bg-white rounded-md shadow-lg w-80 z-20">
                <div className="py-2 px-4 border-b flex justify-between items-center">
                  <h3 className="font-medium text-gray-700">알림</h3>
                  <button 
                    onClick={clearNotifications}
                    className="text-xs text-blue-600 hover:text-blue-800"
                  >
                    모두 읽음 처리
                  </button>
                </div>
                <div className="max-h-96 overflow-y-auto">
                  {notifications.length > 0 ? (
                    <div>
                      {notifications.map(notification => (
                        <div 
                          key={notification.id} 
                          className={`px-4 py-3 hover:bg-gray-50 border-b last:border-0 cursor-pointer ${
                            notification.read ? 'opacity-70' : 'border-l-4 border-l-blue-500'
                          }`}
                          onClick={() => handleNotificationClick(notification)}
                        >
                          <div className="flex items-start">
                            <span className="mr-2 mt-0.5 flex-shrink-0">
                              {getNotificationIcon(notification.type)}
                            </span>
                            <div className="flex-grow min-w-0">
                              <p className="text-sm text-gray-800 mb-1 break-words">{notification.message}</p>
                              <div className="flex justify-between items-center">
                                <p className="text-xs text-gray-500">
                                  {formatTimestamp(notification.time)}
                                </p>
                                {!notification.read && (
                                  <span className="inline-block w-2 h-2 bg-blue-500 rounded-full ml-1 flex-shrink-0"></span>
                                )}
                              </div>
                            </div>
                          </div>
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

const Sidebar = ({ isOpen, toggleSidebar }) => {
  const location = useLocation();
  const healthCheckContext = useHealthCheck();
  const [showStatusModal, setShowStatusModal] = useState(false);
  
  // 디버깅 로그 추가
  // console.log('[레이아웃] 헬스체크 컨텍스트:', healthCheckContext);
  
  // 방어적 프로그래밍: healthCheckContext가 없는 경우를 처리
  const systemStatus = healthCheckContext?.systemStatus || { 
    overall: 'checking', 
    errors: [] 
  };
  const performHealthCheck = healthCheckContext?.performHealthCheck || (() => {
    console.log('[레이아웃] 헬스체크 함수가 존재하지 않습니다.');
  });
  
  // 시스템 상태 변경 로깅
  useEffect(() => {
    // console.log('[레이아웃] 시스템 상태 변경:', systemStatus);
  }, [systemStatus]);
  
  const handleRefreshStatus = () => {
    console.log('[레이아웃] 헬스체크 새로고침 버튼 클릭');
    performHealthCheck();
  };
  
  const menuItems = [
    { name: '대시보드', icon: Home, path: '/' },
    { name: '로봇 관리', icon: CloudLightning, path: '/robots' },
    { name: '고객 관리', icon: Users, path: '/customers' },
    { name: '영상 보기', icon: Video, path: '/video-stream' },
    { name: '주문 관리', icon: ShoppingCart, path: '/orders' },
    { name: '재고 관리', icon: Package, path: '/inventory' },
    { name: '설정', icon: Settings, path: '/settings' },
  ];
  
  // 시스템 상태에 따른 색상과 메시지 결정
  const getStatusInfo = () => {
    switch(systemStatus.overall) {
      case 'healthy':
        return {
          message: '모든 시스템 정상 작동 중',
          color: 'text-green-400'
        };
      case 'unhealthy':
        return {
          message: `시스템 오류: ${systemStatus.errors.length}건`,
          color: 'text-red-400'
        };
      default:
        return {
          message: '시스템 상태 확인 중...',
          color: 'text-yellow-400'
        };
    }
  };
  
  const statusInfo = getStatusInfo();

  return (
    <div className={`sidebar ${isOpen ? 'open' : 'closed'}`}>
      {/* 시스템 상태 모달 */}
      <SystemStatusModal 
        isOpen={showStatusModal} 
        onClose={() => setShowStatusModal(false)} 
        systemStatus={systemStatus}
        onRefresh={handleRefreshStatus}
      />
      
      {/* 모바일 오버레이 */}
      {isOpen && (
        <div 
          className="fixed inset-0 bg-black bg-opacity-50 z-20 lg:hidden"
          onClick={toggleSidebar}
        ></div>
      )}
      
      <aside 
        className={`fixed inset-y-0 left-0 z-30 w-64 bg-gray-800 text-white overflow-y-auto transition-transform transform ${
          isOpen ? 'translate-x-0' : '-translate-x-full'
        } lg:translate-x-0`}
      >
        <div className="p-4">
          <div className="flex items-center justify-between mb-6">
            <h2 className="text-xl font-bold">메뉴</h2>
            <button 
              onClick={toggleSidebar}
              className="p-2 rounded-md hover:bg-gray-700"
            >
              <X size={20} />
            </button>
          </div>
          
          <nav>
            <ul className="space-y-2">
              {menuItems.map((item, index) => {
                const Icon = item.icon;
                return (
                  <li key={index}>
                    <Link
                      to={item.path}
                      className={`flex items-center px-4 py-3 rounded-md ${
                        location.pathname === item.path
                          ? 'bg-blue-600 text-white'
                          : 'text-gray-300 hover:bg-gray-700'
                      }`}
                      onClick={toggleSidebar}
                    >
                      <Icon size={20} className="mr-3" />
                      <span>{item.name}</span>
                    </Link>
                  </li>
                );
              })}
            </ul>
          </nav>
        </div>
        
        <div className="absolute bottom-0 left-0 right-0 p-4 border-t border-gray-700">
          <div className="flex items-center justify-between">
            <div 
              className="flex items-center cursor-pointer hover:bg-gray-700 p-2 rounded-md"
              onClick={() => setShowStatusModal(true)}
            >
              <Monitor size={20} className={`${systemStatus.overall === 'healthy' ? 'text-green-400' : systemStatus.overall === 'unhealthy' ? 'text-red-400' : 'text-yellow-400'} mr-3`} />
              <div>
                <p className="text-sm font-medium text-white">시스템 상태</p>
                <p className={`text-xs ${statusInfo.color}`}>{statusInfo.message}</p>
              </div>
            </div>
            <button 
              onClick={handleRefreshStatus}
              className="p-1 rounded-md text-gray-400 hover:text-white hover:bg-gray-700"
              title="상태 새로고침"
            >
              <RefreshCw size={16} />
            </button>
          </div>
        </div>
      </aside>
    </div>
  );
};

const Layout = ({ children }) => {
  const [sidebarOpen, setSidebarOpen] = useState(false);
  
  const toggleSidebar = () => setSidebarOpen(!sidebarOpen);
  
  return (
    <div className="min-h-screen bg-gray-100">
      <Header toggleSidebar={toggleSidebar} />
      <Sidebar isOpen={sidebarOpen} toggleSidebar={toggleSidebar} />
      
      <main className="pt-14 lg:pl-64 transition-all duration-200 h-[calc(100vh-0.5rem)] overflow-auto">
      {children}
      </main>
    </div>
  );
};

export default Layout; 