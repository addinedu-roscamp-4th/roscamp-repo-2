import React from 'react';
import { NavLink } from 'react-router-dom';
import { 
  Home, 
  Monitor, 
  Users, 
  ShoppingCart, 
  BarChart2, 
  Settings, 
  Server,
  Video,
  AlertTriangle
} from 'react-feather';

// Only include routes that are actually implemented
const navItems = [
  { title: '대시보드', path: '/', icon: Home, isImplemented: true },
  { title: '로봇 관리', path: '/robots', icon: Monitor, isImplemented: false },
  { title: '영상 스트리밍', path: '/video-streams', icon: Video, isImplemented: false },
  { title: '고객·테이블 관리', path: '/customers', icon: Users, isImplemented: false },
  { title: '주문·재고 통계', path: '/stats', icon: BarChart2, isImplemented: false },
  { title: '이벤트·로그', path: '/system', icon: Server, isImplemented: false },
  { title: '비상 상황', path: '/emergency', icon: AlertTriangle, isImplemented: false },
  { title: '설정', path: '/settings', icon: Settings, isImplemented: false },
];

const Sidebar = ({ isOpen }) => {
  // Filter only implemented routes for now
  const availableNavItems = navItems.filter(item => item.isImplemented);
  
  return (
    <aside 
      className={`${
        isOpen ? 'w-64' : 'w-20'
      } transition-all duration-300 bg-gray-800 text-white flex flex-col h-full overflow-hidden`}
    >
      <div className="p-5 flex justify-center">
        <span className={`${!isOpen ? 'hidden' : ''} text-xl font-bold`}>RoboDine</span>
        {!isOpen && <span className="text-xl font-bold">RD</span>}
      </div>

      <nav className="flex-1">
        <ul className="space-y-2 px-3">
          {navItems.map((item) => {
            const isDisabled = !item.isImplemented;
            
            return (
              <li key={item.path} className="mb-2">
                <NavLink
                  to={isDisabled ? '#' : item.path}
                  onClick={e => isDisabled && e.preventDefault()}
                  className={({ isActive }) =>
                    `flex items-center py-3 px-4 rounded-lg ${
                      isActive && !isDisabled
                        ? 'bg-blue-600 text-white'
                        : isDisabled
                          ? 'text-gray-500 cursor-not-allowed opacity-50'
                          : 'text-gray-300 hover:bg-gray-700'
                    }`
                  }
                >
                  <item.icon size={20} />
                  {isOpen && (
                    <span className="ml-4">
                      {item.title}
                      {isDisabled && !isOpen && <span className="text-xs ml-1">(개발중)</span>}
                    </span>
                  )}
                  {isDisabled && isOpen && <span className="text-xs ml-auto">(개발중)</span>}
                </NavLink>
              </li>
            );
          })}
        </ul>
      </nav>

      <div className="p-4 text-xs text-gray-400 border-t border-gray-700">
        {isOpen && <p>RoboDine Admin v1.0</p>}
      </div>
    </aside>
  );
};

export default Sidebar; 