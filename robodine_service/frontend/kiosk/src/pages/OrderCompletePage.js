import React, { useEffect } from 'react';
import { useLocation, useNavigate } from 'react-router-dom';
import Layout from '../components/Layout/Layout';

const OrderCompletePage = () => {
  const navigate = useNavigate();
  const location = useLocation();
  
  // 페이지 상태에서 주문 정보 추출
  const { customerId, totalAmount, paymentMethod } = location.state || {};
  
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

  return (
    <Layout>
      <div className="container mx-auto py-8">
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
          </div>
          
          <p className="text-gray-500 mb-6">이 화면은 10초 후에 자동으로 닫힙니다.</p>
          
          <button 
            className="bg-indigo-600 text-white px-6 py-3 rounded-md hover:bg-indigo-700 transition-colors duration-200"
            onClick={() => navigate('/')}
          >
            메인 메뉴로 이동
          </button>
        </div>
      </div>
    </Layout>
  );
};

export default OrderCompletePage; 