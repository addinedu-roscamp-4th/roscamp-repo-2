import React, { useState, useMemo } from 'react';
import { CheckCircle, XCircle, Clock, AlertTriangle, ChevronRight, DollarSign, ChevronDown } from 'react-feather';

const RecentOrders = ({ orders = [], onViewOrder }) => {
  // 정렬 메뉴 상태
  const [showSortMenu, setShowSortMenu] = useState(false);
  const [selectedSort, setSelectedSort] = useState('STATUS'); // STATUS, ID, PRICE

  // 정렬 옵션 라벨 맵핑
  const sortLabels = {
    STATUS: '상태순',
    ID: '주문 ID순',
    PRICE: '금액순',
  };

  // 정렬된 주문 리스트 계산
  const sortedOrders = useMemo(() => {
    if (!orders) return [];
    const copy = [...orders];
    switch (selectedSort) {
      case 'STATUS':
        return copy.sort((a, b) => {
          // PREPARING 상태 우선
          if (a.status === 'PREPARING' && b.status !== 'PREPARING') return -1;
          if (a.status !== 'PREPARING' && b.status === 'PREPARING') return 1;
          // 이후 ID 내림차순
          return b.id - a.id;
        });
      case 'ID':
        return copy.sort((a, b) => b.id - a.id);
      case 'PRICE':
        return copy.sort((a, b) => b.totalPrice - a.totalPrice);
      default:
        return copy;
    }
  }, [orders, selectedSort]);

  // 아이콘, 클래스, 포맷 함수들
  const getStatusIcon = (status) => {
    switch (status) {
      case 'COMPLETED': return <CheckCircle size={16} className="text-green-500" />;
      case 'CANCELLED': return <XCircle size={16} className="text-red-500" />;
      case 'PENDING': return <Clock size={16} className="text-yellow-500" />;
      case 'PROCESSING': return <AlertTriangle size={16} className="text-blue-500" />;
      default: return <Clock size={16} className="text-gray-500" />;
    }
  };

  const getStatusClass = (status) => {
    switch (status) {
      case 'PLACED': return 'bg-green-100 text-green-800';
      case 'CANCELLED': return 'bg-red-100 text-red-800';
      case 'PREPARING': return 'bg-yellow-100 text-yellow-800';
      case 'SERVED': return 'bg-blue-100 text-blue-800';
      default: return 'bg-gray-100 text-gray-800';
    }
  };

  const formatTimestamp = (timestamp) => {
    const date = new Date(timestamp);
    return new Intl.DateTimeFormat('ko-KR', {
      month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit', hour12: false
    }).format(date);
  };

  const formatPrice = (totalPrice) => {
    return new Intl.NumberFormat('ko-KR', {
      style: 'currency', currency: 'KRW', minimumFractionDigits: 0
    }).format(totalPrice);
  };

  return (
    <div className="bg-white rounded-lg shadow h-full flex flex-col">
      <div className="flex justify-between items-center p-4 border-b">
        <h3 className="font-semibold text-gray-800 flex items-center">
          <DollarSign size={18} className="mr-2 text-blue-500" />최근 주문
        </h3>
        <div className="flex items-center space-x-2 relative">
          <span className="text-sm text-gray-600">정렬 순서</span>
          <button
            className="flex items-center px-3 py-1 border rounded text-sm text-gray-600 hover:bg-gray-100"
            onClick={() => setShowSortMenu(!showSortMenu)}
          >
            {sortLabels[selectedSort]}
            <ChevronDown size={14} className="ml-1" />
          </button>
          {showSortMenu && (
            <div className="absolute right-0 top-full mt-1 w-40 bg-white rounded-md shadow-lg z-20">
              {Object.entries(sortLabels).map(([key, label]) => (
                <button
                  key={key}
                  className={`w-full text-left px-4 py-2 text-sm ${selectedSort === key ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'}`}
                  onClick={() => { setSelectedSort(key); setShowSortMenu(false); }}
                >
                  {label}
                </button>
              ))}
            </div>
          )}
        </div>
      </div>
      <div className="flex-1 overflow-y-auto">
        {sortedOrders.length > 0 ? (
          <table className="min-w-full divide-y divide-gray-200">
            <thead className="bg-gray-50">
              <tr>
                <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">주문 ID</th>
                <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">테이블</th>
                <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">시간</th>
                <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">항목 수</th>
                <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">금액</th>
                <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">상태</th>
                <th className="relative px-6 py-3"><span className="sr-only">상세 보기</span></th>
              </tr>
            </thead>
            <tbody className="bg-white divide-y divide-gray-200">
              {sortedOrders.map(order => (
                <tr key={order.id} className="hover:bg-gray-50">
                  <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900">#{order.id}</td>
                  <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900">테이블 {order.table}</td>
                  <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">{formatTimestamp(order.timestamp)}</td>
                  <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900">{order.totalQuantity}</td>
                  <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">{formatPrice(order.totalPrice)}</td>
                  <td className="px-6 py-4 whitespace-nowrap">
                    <span className={`px-2 inline-flex text-xs leading-5 font-semibold rounded-full ${getStatusClass(order.status)}`}> 
                      {getStatusIcon(order.status)}
                      <span className="ml-1">{order.status}</span>
                    </span>
                  </td>
                  <td className="px-6 py-4 whitespace-nowrap text-right text-sm font-medium">
                    <button onClick={() => onViewOrder(order.id)} className="text-blue-600 hover:text-blue-900 flex items-center justify-end">
                      상세보기<ChevronRight size={16} className="ml-1" />
                    </button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        ) : (
          <div className="flex flex-col items-center justify-center py-10 text-gray-500">
            <p>최근 주문이 없습니다</p>
          </div>
        )}
      </div>
    </div>
  );
};

export default RecentOrders;