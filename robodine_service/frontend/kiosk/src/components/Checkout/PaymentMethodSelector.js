import React from 'react';

const PaymentMethodSelector = ({ selectedMethod, onSelectMethod }) => {
  // 결제 수단 목록
  const paymentMethods = [
    { id: 'card', name: '신용카드', icon: '💳' },
    { id: 'cash', name: '현금', icon: '💵' }
  ];

  return (
    <div className="bg-white p-6 rounded-lg shadow-md mb-6">
      <h2 className="text-xl font-semibold mb-4">결제 수단 선택</h2>
      
      <div className="grid grid-cols-2 gap-4">
        {paymentMethods.map(method => (
          <button
            key={method.id}
            className={`p-4 rounded-lg border-2 flex flex-col items-center justify-center transition-colors duration-200
              ${selectedMethod === method.id 
                ? 'border-indigo-600 bg-indigo-50' 
                : 'border-gray-200 hover:border-indigo-200'}`}
            onClick={() => onSelectMethod(method.id)}
          >
            <span className="text-3xl mb-2">{method.icon}</span>
            <span className="font-medium">{method.name}</span>
          </button>
        ))}
      </div>
    </div>
  );
};

export default PaymentMethodSelector; 