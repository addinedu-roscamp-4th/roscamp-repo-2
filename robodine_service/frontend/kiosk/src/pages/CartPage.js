// CartPage.js
import React, { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import Layout from '../components/Layout/Layout';
import CartItem from '../components/Cart/CartItem';
import TotalSummary from '../components/Cart/CartSummary';
import { useCart } from '../context/CartContext';
import { useUnifiedWebSockets } from '../context/UnifiedWebSocketProvider';

const CartPage = () => {
  const navigate = useNavigate();
  const { 
    cartItems, 
    removeFromCart, 
    increaseQuantity, 
    decreaseQuantity,
    getTotalAmount,
    clearCart
  } = useCart();

  // WebSocket 데이터 사용
  const { data, connected, currentCustomer, customerOrders } = useUnifiedWebSockets();
  const [notifications, setNotifications] = useState([]);
  const [hasActiveOrders, setHasActiveOrders] = useState(false);

  // 현재 고객의 주문 확인
  useEffect(() => {
    if (customerOrders && customerOrders.length > 0) {
      // 아직 완료되지 않은 주문이 있는지 확인
      const activeOrders = customerOrders.filter(order => 
        order['Order.status'] !== 'COMPLETED' && 
        order['Order.status'] !== 'CANCELLED'
      );
      
      setHasActiveOrders(activeOrders.length > 0);
      
      // 진행 중인 주문이 있을 경우 알림 추가
      if (activeOrders.length > 0 && notifications.length === 0) {
        setNotifications([
          ...notifications,
          { id: Date.now(), message: '진행 중인 주문이 있습니다. 상태를 확인해보세요.' }
        ]);
      }
    } else {
      setHasActiveOrders(false);
    }
  }, [customerOrders]);

  // 장바구니 비우기 핸들러
  const handleClearCart = () => {
    if (cartItems.length === 0) return;
    
    clearCart();
    
    // 알림 추가
    setNotifications([
      ...notifications,
      { id: Date.now(), message: '장바구니가 비워졌습니다.' }
    ]);
  };

  // 알림 닫기 핸들러
  const handleCloseNotification = (id) => {
    setNotifications(notifications.filter(n => n.id !== id));
  };

  // 결제 페이지로 이동
  const goToCheckout = () => {
    navigate('/checkout');
  };
  
  // 주문 상태 페이지로 이동
  const goToOrderStatus = () => {
    navigate('/order-status');
  };

  return (
    <Layout notifications={notifications} onCloseNotification={handleCloseNotification}>
      <div className="flex flex-col flex-grow p-6">
        <h1 className="text-3xl font-bold mb-6">장바구니</h1>
        
        {/* 진행 중인 주문 배너 */}
        {hasActiveOrders && (
          <div className="mb-4 p-4 bg-yellow-100 border-l-4 border-yellow-500 text-yellow-700 rounded">
            <div className="flex justify-between items-center">
              <div>
                <p className="font-bold">진행 중인 주문이 있습니다</p>
                <p className="text-sm">주문 현황을 확인해보세요.</p>
              </div>
              <button 
                onClick={goToOrderStatus} 
                className="bg-yellow-500 text-white px-4 py-2 rounded-md"
              >
                주문 확인
              </button>
            </div>
          </div>
        )}
        
        {/* 고객 정보 (있을 경우) */}
        {currentCustomer && (
          <div className="mb-4 p-3 bg-blue-50 border rounded-md text-sm">
            <p>테이블 번호: {localStorage.getItem('kioskTableId') || '1'}</p>
            <p>고객 ID: {currentCustomer}</p>
            <p>주문 내역: {customerOrders?.length || 0}건</p>
          </div>
        )}
        
        <div className="space-y-4 overflow-auto mb-6">
          {cartItems.map(item => (
            <CartItem
              key={item.id}
              item={item}
              onIncrement={() => increaseQuantity(item.id)}
              onDecrement={() => decreaseQuantity(item.id)}
              onRemove={() => removeFromCart(item.id)}
            />
          ))}

          {cartItems.length === 0 && (
            <div className="flex flex-col items-center justify-center h-64 text-gray-500">
              <p className="mb-6 text-2xl">장바구니가 비어 있습니다.</p>
              <button 
                onClick={() => navigate('/')}
                className="bg-[#C49E69] text-white px-6 py-3 rounded-md text-xl"
                aria-label="메뉴 보러가기"
              >
                메뉴 보러가기
              </button>
            </div>
          )}
        </div>

        {cartItems.length > 0 && (
          <TotalSummary
            subtotal={getTotalAmount()}
            className="sticky bottom-0 bg-white py-6"
            proceedButtonClass="bg-[#C49E69] text-white"
            onProceed={goToCheckout}
          />
        )}
      </div>
    </Layout>
  );
};

export default CartPage; 