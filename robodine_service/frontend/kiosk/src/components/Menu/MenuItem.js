import React from 'react';

const MenuItem = ({ item, onAddToCart }) => {

  console.log('MenuItem:', item);
  // 가격을 원화 형식으로 포맷팅
  const formattedPrice = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0
  }).format(item.price);

  // 준비 시간 표시
  const preparationTime = `${item.prepare_time}분`;

  return (
    <div className="bg-white rounded-lg shadow-md overflow-hidden hover:shadow-lg transition-shadow duration-300">
      <div className="h-48 bg-gray-200 flex items-center justify-center">
        {/* 실제 이미지가 있다면 사용하고, 없으면 기본 이미지 표시 */}
        <img
          src={item.image_url || `http://192.168.0.156:8000/images/menu/300x200?text=${encodeURIComponent(item.name)}.png`}
          alt={item.name}
          className="w-full h-full object-cover"
        />
      </div>
      
      <div className="p-4">
        <h3 className="text-lg font-semibold mb-1">{item.name}</h3>
        <div className="flex justify-between items-center mb-2">
          <span className="text-indigo-600 font-bold">{formattedPrice}</span>
          <span className="text-gray-500 text-sm">준비: {preparationTime}</span>
        </div>
        
        <button
          className="w-full py-2 bg-indigo-600 text-white rounded-md hover:bg-indigo-700 transition-colors duration-200"
          onClick={onAddToCart}
        >
          장바구니 담기
        </button>
      </div>
    </div>
  );
};

export default MenuItem; 