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
  Video
} from 'react-feather';

const navItems = [
  { title: '대시보드', path: '/', icon: Home },
  { title: '영상 스트리밍', path: '/video-streams', icon: Video },
  { title: '로봇 관리', path: '/robots', icon: Monitor },
  { title: '고객·테이블 관리', path: '/customers', icon: Users },
  { title: '주문·재고 통계', path: '/stats', icon: BarChart2 },
  { title: '이벤트·로그', path: '/system', icon: Server },
  { title: '설정', path: '/settings', icon: Settings },
];

const Sidebar = ({ isOpen }) => {
  return (
    <aside 
      className={`${
        isOpen ? 'w-64' : 'w-20'
      } transition-width duration-300 bg-gray-800 text-white flex flex-col`}
    >
      <div className="p-5 flex justify-center">
        <span className={`${!isOpen && 'hidden'} text-xl font-bold`}>RoboDine</span>
        {!isOpen && <span className="text-xl font-bold">RD</span>}
      </div>

      <nav className="flex-1">
        <ul>
          {navItems.map((item) => (
            <li key={item.path} className="mb-2">
              <NavLink
                to={item.path}
                className={({ isActive }) =>
                  `flex items-center py-3 px-4 rounded-lg ${
                    isActive
                      ? 'bg-blue-600 text-white'
                      : 'text-gray-300 hover:bg-gray-700'
                  }`
                }
              >
                <item.icon size={20} />
                {isOpen && <span className="ml-4">{item.title}</span>}
              </NavLink>
            </li>
          ))}
        </ul>
      </nav>

      <div className="p-4 text-xs text-gray-400">
        {isOpen && <p>RoboDine Admin v1.0</p>}
      </div>
    </aside>
  );
};

export default Sidebar; 