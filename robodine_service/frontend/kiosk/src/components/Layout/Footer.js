import React from 'react';
import { useNavigate } from 'react-router-dom';
import { useCart } from '../../context/CartContext';

// 네비게이션 항목 컴포넌트
const NavItem = ({ icon, label, onClick }) => {
  return (
    <button
      className="flex flex-col items-center justify-center p-2 w-full h-full 
        hover:bg-indigo-500 transition-colors duration-200 focus:outline-none"
      onClick={onClick}
    >
      <span className="text-2xl">{icon}</span>
      <span className="mt-1 text-sm whitespace-nowrap">{label}</span>
    </button>
  );
};

const Footer = () => {
  const navigate = useNavigate();
  const { getTotalItems } = useCart();

  // 페이지 이동 핸들러
  const goHome = () => navigate('/');
  const goCart = () => navigate('/cart');
  
  // 직원 호출 핸들러
  const callStaff = () => {
    alert('직원을 호출하였습니다. 잠시만 기다려주세요.');
    // 실제 구현에서는 웹소켓이나 API를 통해 직원 호출 기능 구현
  };

  return (
    <footer className="bg-indigo-600 text-white bottom-0 w-full shadow-inner">
      <div className="grid grid-cols-3 divide-x divide-indigo-400 h-16">
        <NavItem 
          icon="🏠" 
          label="홈" 
          onClick={goHome} 
        />
        <NavItem 
          icon="🛒" 
          label={`장바구니 (${getTotalItems()})`} 
          onClick={goCart} 
        />
        <NavItem 
          icon="📢" 
          label="직원 호출" 
          onClick={callStaff} 
        />
      </div>
    </footer>
  );
};

export default Footer; 