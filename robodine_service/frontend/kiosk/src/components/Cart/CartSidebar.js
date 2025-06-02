import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import NotificationOverlay from '../Notifications/NotificationOverlay';
import { useLanguage } from '../../context/LanguageContext';

// 미니 장바구니 아이템 컴포넌트
const MiniCartItem = ({ item, onIncrease, onDecrease, onRemove }) => {
  const { t } = useLanguage();
  
  // 가격 포맷
  const formattedPrice = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0
  }).format(item.price);

  return (
    <div className="flex py-6 border-b border-gray-200">
      {/* 이미지 */}
      <div className="w-28 h-28 bg-gray-100 rounded mr-5 overflow-hidden">
        <img
          src={item.image_url || item.image}
          alt={item.name}
          className="w-full h-full object-cover"
        />
      </div>
      
      {/* 정보 */}
      <div className="flex-grow">
        <div className="flex justify-between">
          <h4 className="font-medium text-xl">{item.name}</h4>
          <button 
            onClick={() => onRemove(item.id)} 
            className="text-gray-400 hover:text-[#E53E3E] text-2xl"
          >
            &times;
          </button>
        </div>
        <div className="flex justify-between items-center mt-3">
          <span className="text-lg font-medium">{formattedPrice}</span>
          
          <div className="flex items-center">
            <button 
              onClick={() => onDecrease(item.id)}
              className="w-12 h-12 flex items-center justify-center border border-gray-300 rounded-l text-2xl font-bold"
            >
              -
            </button>
            <span className="w-14 h-12 flex items-center justify-center border-t border-b border-gray-300 text-xl font-medium">
              {item.quantity}
            </span>
            <button 
              onClick={() => onIncrease(item.id)}
              className="w-12 h-12 flex items-center justify-center border border-gray-300 rounded-r text-2xl font-bold"
            >
              +
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

const CartSidebar = ({ cartItems, totalAmount, onRemove, onIncrease, onDecrease }) => {
  const navigate = useNavigate();
  const [notifications, setNotifications] = useState([]);
  const { t } = useLanguage();

  // 가격 포맷
  const formattedTotalAmount = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0
  }).format(totalAmount);

  // 결제 페이지로 이동
  const handleCheckout = () => {
    if (cartItems.length > 0) {
      navigate('/checkout');
    } else {
      addNotification(t('cart.empty'));
    }
  };

  // 알림 추가 함수
  const addNotification = (message) => {
    const id = Date.now();
    setNotifications(prev => [...prev, { id, message }]);
    
    // 알림 제거 (5초 후)
    setTimeout(() => {
      setNotifications(prev => prev.filter(n => n.id !== id));
    }, 5000);
  };

  // 알림 닫기 핸들러
  const handleCloseNotification = (id) => {
    setNotifications(prev => prev.filter(n => n.id !== id));
  };

  return (
    <div className="w-120 bg-white border-l border-gray-200 flex flex-col">
      <NotificationOverlay 
        notifications={notifications} 
        onClose={handleCloseNotification} 
      />
      <div className="p-6 border-b border-gray-200 bg-[#F7F3EE]">
        <h2 className="text-3xl font-bold">{t('cart.title')}</h2>
      </div>

      {cartItems.length === 0 ? (
        <div className="flex-grow flex flex-col items-center justify-center p-6 text-gray-500">
          <p className="text-2xl">{t('cart.empty')}</p>
        </div>
      ) : (
        <>
          {/* 장바구니 아이템 목록 */}
          <div className="flex-grow overflow-auto p-6">
            {cartItems.map(item => (
              <MiniCartItem
                key={item.id}
                item={item}
                onIncrease={() => {
                  onIncrease(item.id);
                  // addNotification(`${item.name} 수량이 증가했습니다.`);
                }}
                onDecrease={() => {
                  onDecrease(item.id);
                  if (item.quantity > 1) {
                    // addNotification(`${item.name} 수량이 감소했습니다.`);
                  }
                }}
                onRemove={() => {
                  onRemove(item.id);
                  // addNotification(`${item.name}이(가) 장바구니에서 제거되었습니다.`);
                }}
              />
            ))}
          </div>
          
          {/* 합계 및 결제 버튼 */}
          <div className="p-6 border-t border-gray-200">
            <div className="flex justify-between items-center mb-6">
              <span className="font-bold text-2xl">{t('cart.totalPrice')}</span>
              <span className="font-bold text-3xl">{formattedTotalAmount}</span>
            </div>
            <button
              className="w-full py-5 bg-[#C49E69] text-white rounded-md font-semibold text-2xl"
              onClick={handleCheckout}
            >
              {t('cart.checkout')}
            </button>
          </div>
        </>
      )}
    </div>
  );
};

export default CartSidebar; 