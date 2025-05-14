// CartPage.js
import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import Layout from '../components/Layout/Layout';
import CartItem from '../components/Cart/CartItem';
import TotalSummary from '../components/Cart/CartSummary';
import { useCart } from '../context/CartContext';

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

  const [notifications, setNotifications] = useState([]);

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

  return (
    <Layout notifications={notifications} onCloseNotification={handleCloseNotification}>
      <div className="flex flex-col flex-grow p-6">
        <h1 className="text-3xl font-bold mb-6">장바구니</h1>
        
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