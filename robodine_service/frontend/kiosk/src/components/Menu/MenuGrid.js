import React, { useState } from 'react';
import MenuCard from './MenuItem'; // 파일명은 같지만 내부는 MenuCard로 구현됨

const MenuDetailModal = ({ isOpen, onClose, children }) => {
  if (!isOpen) return null;
  
  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 z-50 flex items-center justify-center p-4">
      <div className="bg-white rounded-lg shadow-lg max-w-xl w-full p-8">
        <div className="flex justify-end">
          <button 
            onClick={onClose}
            className="text-gray-500 hover:text-gray-700 text-3xl"
            aria-label="닫기"
          >
            &times;
          </button>
        </div>
        {children}
      </div>
    </div>
  );
};

const MenuGrid = ({ items, onAddToCart, category }) => {
  const [selectedItem, setSelectedItem] = useState(null);
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [quantity, setQuantity] = useState(1);

  const openDetail = (item) => {
    setSelectedItem(item);
    setQuantity(1);
    setIsModalOpen(true);
  };

  const closeModal = () => {
    setIsModalOpen(false);
    setSelectedItem(null);
  };

  const addToCartWithQty = (item, qty) => {
    // 수량과 함께 장바구니에 추가
    for (let i = 0; i < qty; i++) {
      onAddToCart(item);
    }
    closeModal();
  };

  // 메뉴가 없는 경우 메시지 표시
  if (!items || items.length === 0) {
    return (
      <div className="flex justify-center items-center py-8">
        <p className="text-gray-500 text-xl">{category} 카테고리에 메뉴가 없습니다.</p>
      </div>
    );
  }

  return (
    <>
      <div className="grid grid-cols-2 sm:grid-cols-2 lg:grid-cols-3 gap-6 p-6">
        {items.map(item => (
          <MenuCard
            key={item.id}
            item={item}
            onTap={() => openDetail(item)}
            onAdd={() => onAddToCart(item)}
          />
        ))}
      </div>

      {selectedItem && (
        <MenuDetailModal isOpen={isModalOpen} onClose={closeModal}>
          <div className="h-56 w-full flex items-center justify-center bg-gray-50">
            <img 
              src={selectedItem.image_url || selectedItem.image} 
              alt={selectedItem.name} 
              className="h-full object-cover"
            />
          </div>
          <h2 className="text-2xl font-bold mt-5">{selectedItem.name}</h2>
          <p className="text-lg text-gray-600 mt-3">{selectedItem.description || '설명이 없습니다.'}</p>
          
          <div className="flex items-center justify-between mt-6">
            <span className="text-xl font-medium">
              {new Intl.NumberFormat('ko-KR', {
                style: 'currency',
                currency: 'KRW',
                minimumFractionDigits: 0,
              }).format(selectedItem.price)}원
            </span>
            
            <div className="flex items-center">
              <button 
                onClick={() => setQuantity(Math.max(1, quantity - 1))}
                className="w-12 h-12 flex items-center justify-center border border-gray-300 rounded-l text-2xl"
                aria-label="수량 감소"
              >
                -
              </button>
              <span className="w-16 h-12 flex items-center justify-center border-t border-b border-gray-300 text-xl">
                {quantity}
              </span>
              <button 
                onClick={() => setQuantity(quantity + 1)}
                className="w-12 h-12 flex items-center justify-center border border-gray-300 rounded-r text-2xl"
                aria-label="수량 증가"
              >
                +
              </button>
            </div>
          </div>
          
          <button
            className="mt-6 w-full bg-[#C49E69] text-white py-4 rounded-md text-xl font-medium"
            onClick={() => addToCartWithQty(selectedItem, quantity)}
          >
            장바구니에 담기
          </button>
        </MenuDetailModal>
      )}
    </>
  );
};

export default MenuGrid; 