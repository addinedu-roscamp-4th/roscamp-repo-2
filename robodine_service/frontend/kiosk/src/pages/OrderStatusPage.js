import React, { useState, useEffect } from 'react';

const OrderStatusPage = () => {
  // 주문 정보 상태
  const [order, setOrder] = useState({
    id: '12345',
    tableName: 'KIOSK-1',
    createdAt: '2023-10-23 14:30',
    paidAt: '2023-10-23 14:32',
    paymentMethod: '신용카드',
    total: 25000,
    items: [
      { id: 1, name: '아메리카노', qty: 2, status: 'ready', eta: 0 },
      { id: 2, name: '카페라떼', qty: 1, status: 'cooking', eta: 3 },
      { id: 3, name: '치즈케이크', qty: 1, status: 'cooking', eta: 5 },
    ]
  });

  // 실제 구현에서는 API로 주문 상태를 주기적으로 가져옴
  useEffect(() => {
    // 웹소켓 또는 주기적 API 호출을 통한 주문 상태 업데이트 로직 구현
    const timer = setInterval(() => {
      // 데모를 위한 임의 상태 변경
      // 실제 구현에서는 API 호출 결과로 대체
      setOrder(prev => {
        // 랜덤하게 조리중 아이템의 eta 감소
        const updatedItems = prev.items.map(item => {
          if (item.status === 'cooking') {
            // eta가 0이 되면 'ready'로 상태 변경
            if (item.eta <= 1) {
              return { ...item, status: 'ready', eta: 0 };
            }
            // 그렇지 않으면 eta 감소
            return { ...item, eta: item.eta - 1 };
          }
          return item;
        });
        
        return { ...prev, items: updatedItems };
      });
    }, 10000); // 10초마다 업데이트
    
    return () => clearInterval(timer);
  }, []);
  
  return (
    <div className="flex flex-col flex-grow p-6 bg-gray-50 h-full overflow-auto">
      {/* 1) 페이지 헤더 */}
      <h1 className="text-3xl font-bold mb-6">주문 현황</h1>

      {/* 2) 주문 요약 카드 */}
      <div className="bg-white rounded-lg shadow p-6 mb-8">
        <p className="text-xl mb-2"><strong>주문 ID:</strong> #{order.id}</p>
        <p className="text-xl mb-2"><strong>테이블:</strong> {order.tableName}</p>
        <p className="text-lg text-gray-500"><strong>생성 시각:</strong> {order.createdAt}</p>
      </div>

      {/* 3) 메뉴별 준비 상태 */}
      <div className="bg-white rounded-lg shadow p-6 mb-8">
        <h2 className="text-2xl font-medium mb-4">메뉴 준비 상태</h2>
        <div className="space-y-4">
          {order.items.map(item => (
            <div key={item.id} className="flex justify-between items-center py-2">
              <span className="text-xl">{item.name} × {item.qty}</span>
              <span className={
                `px-4 py-2 rounded-full text-lg font-medium ` +
                (item.status === 'ready'
                  ? 'bg-[#48BB78] text-white'
                  : item.status === 'cooking'
                  ? 'bg-[#D69E2E] text-white'
                  : 'bg-gray-200 text-gray-600')
              }>
                {item.status === 'ready'
                  ? '완료'
                  : item.status === 'cooking'
                  ? `조리중 (${item.eta}분)`
                  : '대기중'}
              </span>
            </div>
          ))}
        </div>
      </div>

      {/* 4) 결제 내역 */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-2xl font-medium mb-4">결제 내역</h2>
        <div className="space-y-3 text-xl">
          <p><strong>결제 수단:</strong> {order.paymentMethod}</p>
          <p><strong>총 금액:</strong> {order.total.toLocaleString()}원</p>
          <p className="text-lg text-gray-500"><strong>결제 시각:</strong> {order.paidAt}</p>
        </div>
      </div>
    </div>
  );
};

export default OrderStatusPage; 