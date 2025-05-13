import React from 'react';
import { useNavigate } from 'react-router-dom';

const CartSummary = ({ totalAmount, itemCount }) => {
  const navigate = useNavigate();

  // 가격을 원화 형식으로 포맷팅
  const formattedTotalAmount = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0
  }).format(totalAmount);

  // 결제 페이지로 이동
  const handleProceedToCheckout = () => {
    navigate('/checkout');
  };

  return (
    <div className="bg-white p-6 rounded-lg shadow-md">
      <div className="mb-4 pb-4 border-b">
        <div className="flex justify-between mb-2">
          <span className="text-gray-600">총 {itemCount}개 메뉴</span>
          <span>{formattedTotalAmount}</span>
        </div>
        <div className="flex justify-between font-bold text-lg">
          <span>최종 결제 금액</span>
          <span className="text-indigo-600">{formattedTotalAmount}</span>
        </div>
      </div>
      
      <button
        className="w-full py-3 bg-indigo-600 text-white rounded-md hover:bg-indigo-700 transition-colors duration-200 font-semibold"
        onClick={handleProceedToCheckout}
        disabled={itemCount === 0}
      >
        결제하기
      </button>
      
      {itemCount === 0 && (
        <p className="text-red-500 text-center mt-2 text-sm">
          장바구니에 상품을 담아주세요
        </p>
      )}
    </div>
  );
};

export default CartSummary; 