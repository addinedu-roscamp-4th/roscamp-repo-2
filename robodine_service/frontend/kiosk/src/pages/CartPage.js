// CartPage.js
import React, { useState } from 'react';
import { Link } from 'react-router-dom';
import Layout from '../components/Layout/Layout';
import CartItem from '../components/Cart/CartItem';
import CartSummary from '../components/Cart/CartSummary';
import { useCart } from '../context/CartContext';
import NotificationOverlay from '../components/Notification/NotificationOverlay';

const CartPage = () => {
  const { 
    cartItems, 
    removeFromCart, 
    increaseQuantity, 
    decreaseQuantity,
    getTotalAmount,
    getTotalItems,
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
      { id: Date.now(), type: 'info', message: '장바구니가 비워졌습니다.' }
    ]);
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
        <h1 className="text-2xl font-bold mb-6">장바구니</h1>
        
        {/* 장바구니가 비어있는 경우 */}
        {cartItems.length === 0 ? (
          <div className="bg-white p-6 rounded-lg shadow-md text-center">
            <p className="text-gray-500 mb-4">장바구니가 비어 있습니다.</p>
            <Link 
              to="/" 
              className="inline-block bg-indigo-600 text-white px-6 py-2 rounded-md hover:bg-indigo-700 transition-colors duration-200"
            >
              메뉴 보러가기
            </Link>
          </div>
        ) : (
          <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
            {/* 장바구니 항목 목록 */}
            <div className="md:col-span-2 space-y-4">
              <div className="flex justify-between items-center mb-4">
                <h2 className="text-xl font-semibold">주문 항목</h2>
                <button
                  className="text-red-500 hover:text-red-700"
                  onClick={handleClearCart}
                >
                  장바구니 비우기
                </button>
              </div>
              
              {cartItems.map(item => (
                <CartItem
                  key={item.id}
                  item={item}
                  onIncrease={() => increaseQuantity(item.id)}
                  onDecrease={() => decreaseQuantity(item.id)}
                  onRemove={() => removeFromCart(item.id)}
                />
              ))}
            </div>
            
            {/* 주문 요약 및 결제 버튼 */}
            <div>
              <CartSummary 
                totalAmount={getTotalAmount()} 
                itemCount={getTotalItems()} 
              />
            </div>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default CartPage; 