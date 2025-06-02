import React, { useState } from 'react';
import { useNavigate, useLocation } from 'react-router-dom';
import { useCart } from '../../context/CartContext';

// 직원 호출 모달 컴포넌트
const StaffCallModal = ({ isOpen, onClose, onSelect }) => {
  if (!isOpen) return null;

  const callOptions = [
    { id: 'help', label: '일반 도움요청', icon: '🙋‍♂️' },
    { id: 'menu', label: '메뉴 문의', icon: '🍽️' },
    { id: 'payment', label: '결제 도움', icon: '💳' },
    { id: 'birthday', label: '생일 축하', icon: '🎂' },
    { id: 'other', label: '기타 요청', icon: '❓' }
  ];

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 z-50 flex items-center justify-center p-4">
      <div className="bg-white rounded-lg shadow-lg w-full max-w-lg p-6">
        <div className="flex justify-between items-center mb-6">
          <h2 className="text-2xl font-bold">직원 호출 유형</h2>
          <button 
            onClick={onClose}
            className="text-gray-500 hover:text-gray-700 text-3xl"
            aria-label="닫기"
          >
            &times;
          </button>
        </div>
        
        <div className="grid grid-cols-2 gap-4">
          {callOptions.map(option => (
            <button
              key={option.id}
              className="flex flex-col items-center justify-center bg-gray-100 hover:bg-gray-200 rounded-lg p-6 transition-colors duration-200"
              onClick={() => onSelect(option)}
            >
              <span className="text-4xl mb-2">{option.icon}</span>
              <span className="text-lg font-medium text-center">{option.label}</span>
            </button>
          ))}
        </div>
      </div>
    </div>
  );
};

// 네비게이션 항목 컴포넌트
const NavItem = ({ icon, label, onClick, badge = null, notification = null, isActive }) => {
  return (
    <button
      className={`flex flex-col items-center justify-center p-4 w-full h-full focus:outline-none hover:bg-gray-100 transition-colors duration-200 ${isActive ? 'bg-gray-100 border-b-4 border-[#C49E69]' : ''}`}
      onClick={onClick}
    >
      <div className="relative">
        <span className="text-3xl">{icon}</span>
        {badge && (
          <span className="absolute -top-2 -right-2 bg-[#C49E69] text-white text-sm rounded-full w-6 h-6 flex items-center justify-center">
            {badge}
          </span>
        )}
      </div>
      <span className="mt-2 text-lg font-medium">{label}</span>
      {notification && (
        <span className="mt-1 text-sm text-[#C49E69]">
          {notification}
        </span>
      )}
    </button>
  );
};

const Footer = () => {
  const navigate = useNavigate();
  const location = useLocation();
  const { getTotalItems } = useCart();
  const [staffCalled, setStaffCalled] = useState(false);
  const [hasActiveOrder, setHasActiveOrder] = useState(false);
  const [isModalOpen, setIsModalOpen] = useState(false);

  // 페이지 이동 핸들러
  const goHome = () => navigate('/');
  const goOrderStatus = () => navigate('/order-status');
  
  // 직원 호출 모달 열기
  const openStaffCallModal = () => {
    setIsModalOpen(true);
  };

  // 직원 호출 모달 닫기
  const closeStaffCallModal = () => {
    setIsModalOpen(false);
  };
  
  // 직원 호출 유형 선택 처리
  const handleCallTypeSelect = (option) => {
    setStaffCalled(true);
    closeStaffCallModal();
    alert(`${option.label} 요청이 전달되었습니다. 잠시만 기다려주세요.`);
    
    // 실제 구현에서는 웹소켓이나 API를 통해 직원 호출 기능 구현
    // 호출 유형(option.id)과 함께 서버로 전송
    
    // 10초 후 호출 상태 초기화 (데모용)
    setTimeout(() => {
      setStaffCalled(false);
    }, 10000);
  };

  // 현재 활성화된 경로 확인
  const isActive = (path) => {
    if (path === '/' && location.pathname === '/') return true;
    if (path === '/order-status' && location.pathname === '/order-status') return true;
    return false;
  };

  return (
    <>
      <nav className="w-full bg-white shadow border-b border-gray-200 grid grid-cols-3 divide-x h-24">
        <NavItem 
          icon="🏠" 
          label="홈" 
          onClick={goHome}
          isActive={isActive('/')}
        />
        <NavItem 
          icon="📝" 
          label="주문 현황" 
          notification={hasActiveOrder ? '진행중' : null}
          onClick={goOrderStatus}
          isActive={isActive('/order-status')}
        />
        <NavItem 
          icon="📢" 
          label="직원 호출" 
          notification={staffCalled ? '요청 중' : null}
          onClick={openStaffCallModal}
        />
      </nav>

      <StaffCallModal 
        isOpen={isModalOpen} 
        onClose={closeStaffCallModal} 
        onSelect={handleCallTypeSelect}
      />
    </>
  );
};

export default Footer; 