import React from 'react';
import { useNavigate } from 'react-router-dom';

const TotalSummary = ({ subtotal, className, proceedButtonClass, onProceed }) => {
  const navigate = useNavigate();

  // 가격을 원화 형식으로 포맷팅
  const formattedTotalAmount = new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW',
    minimumFractionDigits: 0
  }).format(subtotal);

  // 결제 페이지로 이동
  const handleProceedToCheckout = () => {
    if (typeof onProceed === 'function') {
      onProceed();
    } else {
      navigate('/checkout');
    }
  };

  return (
    <div className={`bg-white p-8 rounded-lg shadow-md ${className || ''}`}>
      <div className="mb-6 pb-4 border-b">
        <div className="flex justify-between font-bold text-2xl">
          <span>최종 결제 금액</span>
          <span>{formattedTotalAmount}</span>
        </div>
      </div>
      
      <button
        className={`w-full py-4 rounded-md font-semibold text-xl ${proceedButtonClass || 'bg-[#C49E69] text-white'}`}
        onClick={handleProceedToCheckout}
        disabled={subtotal === 0}
        aria-label="결제 진행하기"
      >
        결제하기
      </button>
      
      {subtotal === 0 && (
        <p className="text-[#E53E3E] text-center mt-3 text-lg">
          장바구니에 상품을 담아주세요
        </p>
      )}
    </div>
  );
};

export default TotalSummary; 