import React from 'react';

const MenuCard = ({ item, onTap, onAdd }) => {
  // 가격 포맷
  const formattedPrice = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0,
  }).format(item.price);

  // 메뉴 준비 시간 표시
  const prepareTime = item.prepare_time || '?';

  // 담기 버튼 클릭 처리
  const handleAddToCart = (e) => {
    e.stopPropagation(); // 상위 요소의 클릭 이벤트 전파 방지
    
    // 메뉴 아이템 객체와 고정 수량(1)을 전달
    const itemToAdd = {
      id: item.id,
      name: item.name,
      price: item.price,
      image_url: item.image_url || item.image,
      description: item.description,
      prepare_time: item.prepare_time
    };
    
    onAdd(itemToAdd, 1);
  };

  return (
    <div 
      className="bg-white rounded-lg shadow-md overflow-hidden hover:shadow-lg transition-shadow duration-200"
      onClick={onTap}
      role="button"
      tabIndex={0}
      aria-label={`${item.name} 메뉴 상세 보기`}
    >
      <div className="h-56 w-full flex items-center justify-center bg-gray-50">
        <img
          src={item.image_url || item.image}
          alt={item.name}
          className="h-full object-cover"
        />
      </div>

      <div className="p-5">
        <h3 className="text-2xl font-bold">{item.name}</h3>
        <p className="text-sm text-gray-500 mt-1">예상 조리시간: {prepareTime}분</p>
        <div className="flex justify-between items-center mt-4">
          <span className="text-xl font-medium">{formattedPrice}</span>
          <button
            className="bg-[#C49E69] text-white px-7 py-4 rounded-md text-xl font-bold hover:brightness-95"
            onClick={handleAddToCart}
            aria-label={`${item.name} 장바구니에 담기`}
          >
            담기
          </button>
        </div>
      </div>
    </div>
  );
};

export default MenuCard;
