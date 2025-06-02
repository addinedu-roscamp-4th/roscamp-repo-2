import React, { useState } from 'react';

const CardPaymentForm = () => {
  // 카드 결제 관련 상태
  const [cardNumber, setCardNumber] = useState('');
  const [expiryDate, setExpiryDate] = useState('');
  const [cvc, setCvc] = useState('');
  const [cardHolder, setCardHolder] = useState('');

  // 카드 번호 포맷팅 (4자리마다 공백)
  const formatCardNumber = (value) => {
    const v = value.replace(/\s+/g, '').replace(/[^0-9]/gi, '');
    const matches = v.match(/\d{4,16}/g);
    const match = (matches && matches[0]) || '';
    const parts = [];

    for (let i = 0; i < match.length; i += 4) {
      parts.push(match.substring(i, i + 4));
    }

    if (parts.length) {
      return parts.join(' ');
    } else {
      return value;
    }
  };

  // 카드 번호 입력 핸들러
  const handleCardNumberChange = (e) => {
    const value = e.target.value;
    setCardNumber(formatCardNumber(value));
  };

  // 유효기간 포맷팅 (MM/YY)
  const formatExpiryDate = (value) => {
    const v = value.replace(/\s+/g, '').replace(/[^0-9]/gi, '');
    if (v.length >= 3) {
      return `${v.substring(0, 2)}/${v.substring(2)}`;
    }
    return v;
  };

  // 유효기간 입력 핸들러
  const handleExpiryDateChange = (e) => {
    const value = e.target.value;
    setExpiryDate(formatExpiryDate(value));
  };

  // CVC 입력 핸들러 (3자리 제한)
  const handleCvcChange = (e) => {
    const value = e.target.value.replace(/\D/g, '');
    if (value.length <= 3) {
      setCvc(value);
    }
  };

  return (
    <div className="bg-white p-6 rounded-lg shadow-md">
      <h2 className="text-2xl font-semibold mb-4">카드 정보 입력</h2>
      
      <div className="space-y-6">
        {/* 카드 번호 입력 */}
        <div>
          <label htmlFor="cardNumber" className="block text-xl text-gray-700 mb-2">카드 번호</label>
          <input
            id="cardNumber"
            type="text"
            className="w-full p-4 text-xl border border-gray-300 rounded-md focus:ring-indigo-500 focus:border-indigo-500"
            placeholder="1234 5678 9012 3456"
            value={cardNumber}
            onChange={handleCardNumberChange}
            maxLength={19} // 16자리 숫자 + 3개 공백
          />
        </div>
        
        {/* 카드 소유자명 */}
        <div>
          <label htmlFor="cardHolder" className="block text-xl text-gray-700 mb-2">카드 소유자명</label>
          <input
            id="cardHolder"
            type="text"
            className="w-full p-4 text-xl border border-gray-300 rounded-md focus:ring-indigo-500 focus:border-indigo-500"
            placeholder="홍길동"
            value={cardHolder}
            onChange={(e) => setCardHolder(e.target.value)}
          />
        </div>
        
        {/* 유효기간 및 CVC */}
        <div className="grid grid-cols-2 gap-4">
          <div>
            <label htmlFor="expiryDate" className="block text-xl text-gray-700 mb-2">유효기간 (MM/YY)</label>
            <input
              id="expiryDate"
              type="text"
              className="w-full p-4 text-xl border border-gray-300 rounded-md focus:ring-indigo-500 focus:border-indigo-500"
              placeholder="MM/YY"
              value={expiryDate}
              onChange={handleExpiryDateChange}
              maxLength={5} // MM/YY 형식
            />
          </div>
          <div>
            <label htmlFor="cvc" className="block text-xl text-gray-700 mb-2">보안코드 (CVC)</label>
            <input
              id="cvc"
              type="text"
              className="w-full p-4 text-xl border border-gray-300 rounded-md focus:ring-indigo-500 focus:border-indigo-500"
              placeholder="123"
              value={cvc}
              onChange={handleCvcChange}
              maxLength={3}
            />
          </div>
        </div>
      </div>
    </div>
  );
};

export default CardPaymentForm; 