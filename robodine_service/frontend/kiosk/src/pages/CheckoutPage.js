import React, { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import Sidebar from '../components/Layout/Sidebar';
import PaymentMethodSelector from '../components/Checkout/PaymentMethodSelector';
import CardPaymentForm from '../components/Checkout/CardPaymentForm';
import { useCart } from '../context/CartContext';
import { useUnifiedWebSockets } from '../context/UnifiedWebSocketProvider';
import { createCustomer, createOrder } from '../api/orderApi';
import NotificationOverlay from '../components/Notifications/NotificationOverlay';

const CheckoutPage = () => {
  const navigate = useNavigate();
  const { cartItems, getTotalAmount, prepareOrderData, clearCart } = useCart();
  const { data, connected, currentCustomer } = useUnifiedWebSockets();
  
  // 상태 관리
  const [paymentMethod, setPaymentMethod] = useState('card'); // 기본값: 카드 결제
  const [customerCount, setCustomerCount] = useState(1); // 기본값: 1명
  const [isProcessing, setIsProcessing] = useState(false);
  const [error, setError] = useState('');
  const [notifications, setNotifications] = useState([]);
  const [selectedCategory, setSelectedCategory] = useState('추천');

  // 테이블 ID는 localStorage에서 가져오기
  const tableId = parseInt(localStorage.getItem('kioskTableId') || '1');
  
  // 장바구니 검증
  useEffect(() => {
    if (cartItems.length === 0) {
      setNotifications([
        { id: Date.now(), message: '장바구니가 비어 있습니다. 메뉴를 선택해주세요.' }
      ]);
      
      // 3초 후 메뉴 페이지로 리다이렉트
      const timer = setTimeout(() => {
        navigate('/');
      }, 3000);
      
      return () => clearTimeout(timer);
    }
  }, [cartItems, navigate]);

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

  // 알림 닫기 핸들러
  const handleCloseNotification = (id) => {
    setNotifications(notifications.filter(n => n.id !== id));
  };

  // 알림 추가 함수
  const addNotification = (message) => {
    const id = Date.now();
    setNotifications(prev => [...prev, { id, message }]);
  };

  // 결제 처리 핸들러
  const handlePayment = async () => {
    try {
      // 이미 처리 중이면 중복 요청 방지
      if (isProcessing) return;
      
      setIsProcessing(true);
      setError('');
      
      // 장바구니가 비어있는지 확인
      if (cartItems.length === 0) {
        setError('장바구니가 비어 있습니다.');
        addNotification('장바구니가 비어 있습니다.');
        setIsProcessing(false);
        return;
      }
      
      let orderId, customerId;
      
      // 고객 정보 생성 또는 기존 고객 정보 사용
      if (currentCustomer) {
        // 이미 테이블에 고객이 배정되어 있으면 해당 고객 ID 사용
        customerId = currentCustomer;
      } else {
        // 새 고객 생성
        try {
          const customerResponse = await createCustomer(customerCount);
          console.log('고객 정보 생성 응답:', customerResponse);
          
          customerId = customerResponse.customer_id;
          if (!customerId) {
            throw new Error('고객 ID가 없습니다');
          }
        } catch (err) {
          console.error('고객 정보 생성 실패:', err);
          setError('고객 정보 생성에 실패했습니다. 다시 시도해 주세요.');
          setIsProcessing(false);
          return;
        }
      }
      
      // 주문 생성
      try {
        // 주문 데이터 준비
        const orderData = {
          customer_id: customerId,
          table_id: tableId,
          items: cartItems.map(item => ({
            menu_item_id: item.id,
            quantity: item.quantity
          }))
        };
        
        const orderResponse = await createOrder(orderData);
        console.log('주문 생성 응답:', orderResponse);
        
        orderId = orderResponse.order_id;
      } catch (err) {
        console.error('주문 생성 실패:', err);
        setError('주문 생성에 실패했습니다. 다시 시도해 주세요.');
        setIsProcessing(false);
        return;
      }
      
      // 결제 시각 저장
      const paymentTime = new Date().toISOString();
      
      // localStorage에 결제 시각 저장
      localStorage.setItem('lastPaymentTime', paymentTime);
      
      // 성공 메시지 추가
      addNotification('결제가 완료되었습니다! 주문 처리 중입니다.');
      
      // 장바구니 비우기
      clearCart();
      
      // 성공 메시지 및 결제 완료 페이지로 이동
      navigate('/order-complete', { 
        state: { 
          orderId,
          customerId,
          tableId,
          totalAmount: getTotalAmount(),
          paymentMethod,
          paymentTime
        } 
      });
      
    } catch (err) {
      console.error('결제 처리 중 오류:', err);
      setError('결제 처리 중 오류가 발생했습니다. 다시 시도해 주세요.');
      addNotification('결제 처리 중 오류가 발생했습니다.');
      setIsProcessing(false);
    }
  };

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
      <div className="container mx-auto py-8 h-full overflow-auto flex-grow">
        <h1 className="text-3xl font-bold mb-6 px-6">결제</h1>
        
        {error && (
          <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4 text-xl mx-6">
            {error}
          </div>
        )}
        
        {/* 고객 정보 및 테이블 표시 */}
        <div className="bg-blue-50 p-4 rounded-lg mb-6 mx-6">
          <h2 className="text-xl font-semibold mb-2">주문 정보</h2>
          <p className="text-lg">테이블 번호: {tableId}</p>
          {currentCustomer && (
            <p className="text-lg">고객 ID: {currentCustomer} (기존 고객)</p>
          )}
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6 px-6">
          {/* 결제 정보 폼 섹션 */}
          <div className="md:col-span-2 space-y-6">
            {/* 인원 수 입력 - 이미 고객이 있으면 비활성화 */}
            <div className="bg-white p-6 rounded-lg shadow-md">
              <h2 className="text-2xl font-semibold mb-4">인원 수</h2>
              <div>
                <label htmlFor="customerCount" className="block text-xl text-gray-700 mb-2">방문 인원 수</label>
                <select
                  id="customerCount"
                  className={`w-full p-4 text-xl border rounded-md ${currentCustomer ? 'bg-gray-100' : ''}`}
                  value={customerCount}
                  onChange={handleCustomerCountChange}
                  disabled={currentCustomer}
                >
                  {[1, 2, 3, 4, 5, 6, 7, 8, 9, 10].map(num => (
                    <option key={num} value={num}>{num}명</option>
                  ))}
                </select>
                {currentCustomer && (
                  <p className="mt-2 text-gray-500">이미 테이블에 고객이 등록되어 있습니다.</p>
                )}
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
              <h2 className="text-2xl font-semibold mb-4">주문 요약</h2>
              
              <div className="mb-4 pb-4 border-b">
                <div className="space-y-3">
                  {cartItems.map(item => (
                    <div key={item.id} className="flex justify-between text-xl">
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
              
              <div className="flex justify-between font-bold mb-8">
                <span className="text-2xl">총 결제 금액</span>
                <span className="text-3xl text-indigo-600">{formattedTotalAmount}</span>
              </div>
              
              <button
                className={`w-full py-6 rounded-md font-bold text-2xl text-white 
                  ${isProcessing 
                    ? 'bg-gray-400 cursor-not-allowed' 
                    : 'bg-[#C49E69] hover:bg-[#C49E00] transition-colors duration-200'
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
    </div>
  );
};

export default CheckoutPage; 