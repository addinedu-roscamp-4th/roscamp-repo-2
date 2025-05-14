import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import Layout from '../components/Layout/Layout';
import PaymentMethodSelector from '../components/Checkout/PaymentMethodSelector';
import CardPaymentForm from '../components/Checkout/CardPaymentForm';
import NotificationOverlay from '../components/Notification/NotificationOverlay';
import { useCart } from '../context/CartContext';
import { createCustomer, createOrder } from '../api/orderApi';

const CheckoutPage = () => {
  const navigate = useNavigate();
  const { cartItems, getTotalAmount, prepareOrderData, clearCart } = useCart();
  
  // 상태 관리
  const [paymentMethod, setPaymentMethod] = useState('card'); // 기본값: 카드 결제
  const [customerCount, setCustomerCount] = useState(1); // 기본값: 1명
  const [isProcessing, setIsProcessing] = useState(false);
  const [notifications, setNotifications] = useState([]);

  // 총 주문 금액 포맷팅
  const formattedTotalAmount = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0
  }).format(getTotalAmount());

  // 결제 방법 선택 핸들러
  const handlePaymentMethodChange = (method) => {
    setPaymentMethod(method);
  };

  // 인원 수 변경 핸들러
  const handleCustomerCountChange = (e) => {
    const count = parseInt(e.target.value);
    if (!isNaN(count) && count > 0 && count <= 10) {
      setCustomerCount(count);
    }
  };

  // 결제 처리 핸들러
  const handlePayment = async () => {
    try {
      // 이미 처리 중이면 중복 요청 방지
      if (isProcessing) return;
      
      setIsProcessing(true);
      
      // 장바구니가 비어있는지 확인
      if (cartItems.length === 0) {
        setNotifications([
          ...notifications,
          { id: Date.now(), type: 'error', message: '장바구니가 비어 있습니다.' }
        ]);
        setIsProcessing(false);
        return;
      }
      
      // 1. 고객 정보 생성 (인원 수)
      const customerResponse = await createCustomer(customerCount);
      console.log('response:', customerResponse);
      const customerId = customerResponse.customer_id;
      if (!customerId) {
        throw new Error('고객 정보 생성 실패');
      }
      const tableId = customerResponse.tableId;
      if (!tableId) {
        throw new Error('테이블 정보 생성 실패');
      }
      
      // 2. 주문 데이터 준비 및 생성
      const orderData = prepareOrderData(customerId, tableId);
      await createOrder(orderData);
      
      // 3. 장바구니 비우기
      clearCart();
      
      // 4. 성공 메시지 및 결제 완료 페이지로 이동
      navigate('/order-complete', { 
        state: { 
          customerId,
          totalAmount: getTotalAmount(),
          paymentMethod 
        } 
      });
      
    } catch (err) {
      console.error('결제 처리 중 오류:', err);
      
      // 오류 알림 표시
      setNotifications([
        ...notifications,
        { id: Date.now(), type: 'error', message: '결제 처리 중 오류가 발생했습니다. 다시 시도해 주세요.' }
      ]);
      
      setIsProcessing(false);
    }
  };

  // 알림 닫기 핸들러
  const handleCloseNotification = (id) => {
    setNotifications(notifications.filter(n => n.id !== id));
  };

  return (
    <Layout>
      {/* 알림 오버레이 */}
      <NotificationOverlay 
        notifications={notifications}
        onClose={handleCloseNotification}
      />
      
      <div className="container mx-auto py-8">
        <h1 className="text-2xl font-bold mb-6">결제</h1>
        
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          {/* 결제 정보 폼 섹션 */}
          <div className="md:col-span-2 space-y-6">
            {/* 인원 수 입력 */}
            <div className="bg-white p-6 rounded-lg shadow-md">
              <h2 className="text-xl font-semibold mb-4">인원 수</h2>
              <div>
                <label htmlFor="customerCount" className="block text-gray-700 mb-2">방문 인원 수</label>
                <select
                  id="customerCount"
                  className="w-full p-3 border border-gray-300 rounded-md focus:ring-indigo-500 focus:border-indigo-500"
                  value={customerCount}
                  onChange={handleCustomerCountChange}
                >
                  {[1, 2, 3, 4, 5, 6, 7, 8, 9, 10].map(num => (
                    <option key={num} value={num}>{num}명</option>
                  ))}
                </select>
              </div>
            </div>
            
            {/* 결제 방법 선택 */}
            <PaymentMethodSelector 
              selectedMethod={paymentMethod}
              onSelectMethod={handlePaymentMethodChange}
            />
            
            {/* 카드 결제 선택 시 카드 정보 입력 폼 표시 */}
            {paymentMethod === 'card' && <CardPaymentForm />}
          </div>
          
          {/* 주문 요약 및 결제 버튼 */}
          <div>
            <div className="bg-white p-6 rounded-lg shadow-md">
              <h2 className="text-xl font-semibold mb-4">주문 요약</h2>
              
              <div className="mb-4 pb-4 border-b">
                <div className="space-y-2">
                  {cartItems.map(item => (
                    <div key={item.id} className="flex justify-between">
                      <span>{item.name} x {item.quantity}</span>
                      <span>
                        {new Intl.NumberFormat('ko-KR', {
                          style: 'currency',
                          currency: 'KRW',
                          minimumFractionDigits: 0
                        }).format(item.price * item.quantity)}
                      </span>
                    </div>
                  ))}
                </div>
              </div>
              
              <div className="flex justify-between font-bold text-lg mb-6">
                <span>총 결제 금액</span>
                <span className="text-indigo-600">{formattedTotalAmount}</span>
              </div>
              
              <button
                className={`w-full py-3 rounded-md font-semibold text-white 
                  ${isProcessing 
                    ? 'bg-gray-400 cursor-not-allowed' 
                    : 'bg-indigo-600 hover:bg-indigo-700 transition-colors duration-200'
                  }`}
                onClick={handlePayment}
                disabled={isProcessing}
              >
                {isProcessing ? '처리 중...' : '결제 완료'}
              </button>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default CheckoutPage; 