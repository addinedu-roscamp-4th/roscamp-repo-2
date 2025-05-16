import React, { useEffect, useState } from 'react';
import { useLocation, useNavigate } from 'react-router-dom';
import Sidebar from '../components/Layout/Sidebar';
import NotificationOverlay from '../components/Notifications/NotificationOverlay';

const OrderCompletePage = () => {
  const navigate = useNavigate();
  const location = useLocation();
  const [selectedCategory, setSelectedCategory] = useState('추천');
  const [notifications, setNotifications] = useState([]);
  
  // 페이지 상태에서 주문 정보 추출
  const { customerId, totalAmount, paymentMethod, paymentTime } = location.state || {};
  
  // 알림 추가 함수
  const addNotification = (message) => {
    const id = Date.now();
    setNotifications(prev => [...prev, { id, message }]);
  };

  // 알림 닫기 처리
  const handleCloseNotification = (id) => {
    setNotifications(notifications.filter(n => n.id !== id));
  };
  
  // 총 주문 금액 포맷팅
  const formattedTotalAmount = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0
  }).format(totalAmount || 0);
  
  // 주문 번호 생성 (실제로는 서버에서 받아와야 함)
  const orderNumber = customerId ? `ORD-${customerId}` : 'ORD-UNKNOWN';
  
  // 결제 방법 텍스트
  const paymentMethodText = paymentMethod === 'card' ? '신용카드' : '현금';
  
  // 결제 시간 표시
  const formattedPaymentTime = paymentTime ? new Date(paymentTime).toLocaleString('ko-KR', {
    year: 'numeric',
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
    hour12: false
  }) : '-';
  
  // 10초 후 홈으로 자동 이동
  useEffect(() => {
    const timer = setTimeout(() => {
      navigate('/');
    }, 10000);
    
    return () => clearTimeout(timer);
  }, [navigate]);
  
  // 고객 ID가 없는 경우 홈으로 이동 (직접 URL 접근 방지)
  useEffect(() => {
    if (!customerId) {
      navigate('/');
    }
  }, [customerId, navigate]);

  // 주문 완료 메시지 표시
  useEffect(() => {
    if (customerId) {
      addNotification('주문이 완료되었습니다! 음식이 준비되면 서빙됩니다.');
    }
  }, [customerId]);

  return (
    <div className="flex h-screen">
      <NotificationOverlay 
        notifications={notifications} 
        onClose={handleCloseNotification} 
      />
      <Sidebar 
        selectedCategory={selectedCategory}
        onSelectCategory={setSelectedCategory}
      />
      <div className="flex-grow py-8 overflow-auto">
        <div className="bg-white p-8 rounded-lg shadow-md max-w-2xl mx-auto text-center">
          <div className="mb-6">
            <span className="text-5xl text-green-500">✓</span>
            <h1 className="text-2xl font-bold mt-4 mb-2">주문이 완료되었습니다!</h1>
            <p className="text-gray-600">
              주문해 주셔서 감사합니다. 음식이 준비되면 서빙 로봇이 배달해 드립니다.
            </p>
          </div>
          
          <div className="border-t border-b py-6 my-6">
            <div className="flex justify-between py-2">
              <span className="font-medium">주문 번호:</span>
              <span>{orderNumber}</span>
            </div>
            <div className="flex justify-between py-2">
              <span className="font-medium">결제 방법:</span>
              <span>{paymentMethodText}</span>
            </div>
            <div className="flex justify-between py-2">
              <span className="font-medium">결제 금액:</span>
              <span className="text-indigo-600 font-bold">{formattedTotalAmount}</span>
            </div>
            <div className="flex justify-between py-2">
              <span className="font-medium">결제 시각:</span>
              <span>{formattedPaymentTime}</span>
            </div>
          </div>
          
          <p className="text-gray-500 mb-6">이 화면은 10초 후에 자동으로 닫힙니다.</p>
          
          <button 
            className="bg-[#C49E69] text-white px-6 py-3 rounded-md hover:bg-[#C49E00] transition-colors duration-200"
            onClick={() => navigate('/')}
          >
            메인 메뉴로 이동
          </button>
        </div>
      </div>
    </div>
  );
};

export default OrderCompletePage; 