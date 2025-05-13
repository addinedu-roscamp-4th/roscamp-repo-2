import React from 'react';

// 카테고리 버튼 컴포넌트
const CategoryButton = ({ label, selected, onClick }) => {
  return (
    <button
      className={`w-full py-4 px-6 text-left text-lg font-medium transition-colors duration-200 
        ${selected 
          ? 'bg-indigo-600 text-white' 
          : 'bg-white text-gray-700 hover:bg-indigo-100'
        }`}
      onClick={onClick}
    >
      {label}
    </button>
  );
};

const Sidebar = ({ selectedCategory, onSelectCategory }) => {
  // 카테고리 목록
  const categories = [
    { id: 'recommended', label: '추천 메뉴' },
    { id: 'food', label: '음식' },
    { id: 'beverage', label: '음료' }
  ];

  return (
    <aside className="w-64 bg-white shadow-md">
      <div className="flex flex-col h-full">
        {categories.map(category => (
          <CategoryButton
            key={category.id}
            label={category.label}
            selected={selectedCategory === category.label}
            onClick={() => onSelectCategory(category.label)}
          />
        ))}
      </div>
    </aside>
  );
};

export default Sidebar; 