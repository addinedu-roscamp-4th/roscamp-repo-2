import React from 'react';

const MenuItem = ({ item, onAddToCart }) => {
  // 가격 포맷
  const formattedPrice = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0,
  }).format(item.price);

  // 준비 시간
  const preparationTime = `${item.prepare_time}분`;

  return (
    <div className="bg-white rounded-lg shadow-md overflow-hidden hover:shadow-lg transition-shadow duration-300">
      {/* 이미지 컨테이너 수정 - 고정 크기와 중앙 정렬 */}
      <div className="h-[200px] w-full bg-gray-100 flex items-center justify-center">
        <img
          src={item.image_url}
          alt={item.name}
          className="max-h-[180px] max-w-[270px] object-contain"
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
