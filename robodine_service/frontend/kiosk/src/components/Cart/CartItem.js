import React from 'react';

const CartItem = ({ item, onIncrease, onDecrease, onRemove }) => {

  // 가격을 원화 형식으로 포맷팅
  const formattedPrice = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0
  }).format(item.price);

  // 항목 총 가격 계산 및 포맷팅
  const totalPrice = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0
  }).format(item.price * item.quantity);

  return (
    <div className="flex items-center p-4 bg-white rounded-lg shadow mb-3">
      {/* 메뉴 이미지 */}
      <div className="w-16 h-16 bg-gray-200 rounded mr-4 overflow-hidden">
        <img
          src={item.image_url || `http://192.168.0.156:8000/images/menu/100?text=${encodeURIComponent(item.name)}.png`}
          alt={item.name}
          className="w-full h-full object-cover"
        />
      </div>
      
      {/* 메뉴 이름 및 단가 */}
      <div className="flex-grow">
        <h3 className="font-medium">{item.name}</h3>
        <p className="text-gray-600 text-sm">{formattedPrice} / 개</p>
      </div>
      
      {/* 수량 조절 */}
      <div className="flex items-center mr-4">
        <button
          className="w-8 h-8 rounded-full bg-gray-200 flex items-center justify-center hover:bg-gray-300"
          onClick={onDecrease}
        >
          -
        </button>
        <span className="mx-2 w-6 text-center">{item.quantity}</span>
        <button
          className="w-8 h-8 rounded-full bg-gray-200 flex items-center justify-center hover:bg-gray-300"
          onClick={onIncrease}
        >
          +
        </button>
      </div>
      
      {/* 항목 총 가격 */}
      <div className="w-24 text-right font-semibold">
        {totalPrice}
      </div>
      
      {/* 삭제 버튼 */}
      <button
        className="ml-4 text-red-500 hover:text-red-700"
        onClick={onRemove}
      >
        삭제
      </button>
    </div>
  );
};

export default CartItem; 