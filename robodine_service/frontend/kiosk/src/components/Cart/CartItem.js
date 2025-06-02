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
    <div className="flex items-center p-6 bg-white rounded-lg shadow mb-4">
      {/* 메뉴 이미지 */}
      <div className="w-24 h-24 bg-gray-100 rounded mr-6 overflow-hidden">
        <img
          src={item.image_url}
          alt={item.name}
          className="w-full h-full object-cover"
        />
      </div>
      
      {/* 메뉴 이름 및 단가 */}
      <div className="flex-grow">
        <h3 className="text-xl font-medium">{item.name}</h3>
        <p className="text-gray-600 text-lg">{formattedPrice} / 개</p>
      </div>
      
      {/* 수량 조절 */}
      <div className="flex items-center mr-6">
        <button
          className="w-12 h-12 flex items-center justify-center border border-gray-300 rounded-l text-2xl"
          onClick={onDecrease}
          aria-label="수량 감소"
        >
          -
        </button>
        <span className="w-16 h-12 flex items-center justify-center border-t border-b border-gray-300 text-xl">
          {item.quantity}
        </span>
        <button
          className="w-12 h-12 flex items-center justify-center border border-gray-300 rounded-r text-2xl"
          onClick={onIncrease}
          aria-label="수량 증가"
        >
          +
        </button>
      </div>
      
      {/* 항목 총 가격 */}
      <div className="w-32 text-right font-medium text-xl">
        {totalPrice}
      </div>
      
      {/* 삭제 버튼 */}
      <button
        className="ml-6 border border-[#E53E3E] text-[#E53E3E] px-4 py-2 rounded-md text-lg"
        onClick={onRemove}
        aria-label={`${item.name} 삭제`}
      >
        삭제
      </button>
    </div>
  );
};

export default CartItem; 