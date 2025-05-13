import React from 'react';
import MenuItem from './MenuItem';

const MenuGrid = ({ items, onAddToCart }) => {
  // 메뉴가 없는 경우 메시지 표시
  if (!items || items.length === 0) {
    return (
      <div className="flex justify-center items-center h-full">
        <p className="text-gray-500 text-xl">해당 카테고리에 메뉴가 없습니다.</p>
      </div>
    );
  }

  return (
    <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 gap-6">
      {items.map(item => (
        <MenuItem
          key={item.id}
          item={item}
          onAddToCart={() => onAddToCart(item)}
        />
      ))}
    </div>
  );
};

export default MenuGrid; 