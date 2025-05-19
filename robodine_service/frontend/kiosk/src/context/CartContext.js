// CartContext.js
import React, { createContext, useState, useContext, useEffect } from 'react';

// 장바구니 컨텍스트 생성
const CartContext = createContext();

// 장바구니 컨텍스트 제공자 컴포넌트
export const CartProvider = ({ children }) => {
  // 로컬 스토리지에서 장바구니 데이터 로드 (있으면)
  const [cartItems, setCartItems] = useState(() => {
    const savedCart = localStorage.getItem('kioskCart');
    return savedCart ? JSON.parse(savedCart) : [];
  });

  // 장바구니 변경시 로컬 스토리지에 저장
  useEffect(() => {
    localStorage.setItem('kioskCart', JSON.stringify(cartItems));
  }, [cartItems]);

  // 장바구니에 항목 추가
  const addToCart = (item) => {
    setCartItems(prevItems => {
      // 이미 장바구니에 있는지 확인
      const existingItemIndex = prevItems.findIndex(
        cartItem => cartItem.id === item.id
      );

      if (existingItemIndex !== -1) {
        // 있으면 수량 증가
        const updatedItems = [...prevItems];
        updatedItems[existingItemIndex].quantity += 1;
        return updatedItems;
      } else {
        // 없으면 새로 추가 (수량 1로 시작)
        return [...prevItems, { ...item, quantity: 1 }];
      }
    });
  };

  // 장바구니에서 항목 제거
  const removeFromCart = (itemId) => {
    setCartItems(prevItems => 
      prevItems.filter(item => item.id !== itemId)
    );
  };

  // 장바구니 항목 수량 증가
  const increaseQuantity = (itemId) => {
    setCartItems(prevItems => 
      prevItems.map(item => 
        item.id === itemId 
          ? { ...item, quantity: item.quantity + 1 } 
          : item
      )
    );
  };

  // 장바구니 항목 수량 감소 (1 미만이면 항목 제거)
  const decreaseQuantity = (itemId) => {
    setCartItems(prevItems => 
      prevItems.map(item => 
        item.id === itemId && item.quantity > 1
          ? { ...item, quantity: item.quantity - 1 } 
          : item
      ).filter(item => !(item.id === itemId && item.quantity <= 1))
    );
  };

  // 장바구니 비우기
  const clearCart = () => {
    setCartItems([]);
  };

  // 장바구니 총 금액 계산
  const getTotalAmount = () => {
    return cartItems.reduce(
      (total, item) => total + item.price * item.quantity, 
      0
    );
  };

  // 장바구니 총 수량 계산
  const getTotalItems = () => {
    return cartItems.reduce(
      (total, item) => total + item.quantity, 
      0
    );
  };

  // API 호출을 위한 주문 데이터 형식으로 변환
  const prepareOrderData = (customerId) => {
    return {
      customer_id: customerId,
      items: cartItems.map(item => ({
        menu_item_id: item.id,
        quantity: item.quantity
      }))
    };
  };

  // 컨텍스트 값으로 제공할 상태와 함수들
  const value = {
    cartItems,
    addToCart,
    removeFromCart,
    increaseQuantity,
    decreaseQuantity,
    clearCart,
    getTotalAmount,
    getTotalItems,
    prepareOrderData
  };

  return (
    <CartContext.Provider value={value}>
      {children}
    </CartContext.Provider>
  );
};

// 컨텍스트 사용을 위한 커스텀 훅
export const useCart = () => {
  const context = useContext(CartContext);
  if (!context) {
    throw new Error('useCart는 CartProvider 내부에서만 사용할 수 있습니다');
  }
  return context;
}; 