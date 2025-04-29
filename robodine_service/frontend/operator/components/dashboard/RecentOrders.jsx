import React from 'react';
import { AlertTriangle, Eye, Clock, DollarSign, CheckCircle, Truck, XCircle } from 'react-feather';

const RecentOrders = ({ orders, onViewOrder, error }) => {
  // 에러 상태 처리
  if (error) {
    return (
      <div className="bg-white p-4 rounded-lg shadow">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">최근 주문</h2>
        <div className="bg-red-50 border border-red-200 text-red-700 px-4 py-8 rounded flex items-center justify-center">
          <AlertTriangle className="mr-2" size={20} />
          <p>{error}</p>
        </div>
      </div>
    );
  }

  // 데이터가 없는 경우 처리
  if (!orders || orders.length === 0) {
    return (
      <div className="bg-white p-4 rounded-lg shadow">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">최근 주문</h2>
        <div className="text-gray-500 p-4 text-center">
          최근 주문이 없습니다
        </div>
      </div>
    );
  }

  // 주문 상태에 따른 아이콘 가져오기
  const getStatusIcon = (status) => {
    switch (status) {
      case 'COMPLETED':
        return <CheckCircle className="text-green-500" size={18} />;
      case 'PENDING':
        return <Clock className="text-yellow-500" size={18} />;
      case 'CANCELLED':
        return <XCircle className="text-red-500" size={18} />;
      case 'DELIVERING':
        return <Truck className="text-blue-500" size={18} />;
      default:
        return <Clock className="text-gray-500" size={18} />;
    }
  };

  // 주문 상태에 따른 배지 색상 가져오기
  const getStatusBadgeClass = (status) => {
    switch (status) {
      case 'COMPLETED':
        return 'bg-green-100 text-green-800';
      case 'PENDING':
        return 'bg-yellow-100 text-yellow-800';
      case 'CANCELLED':
        return 'bg-red-100 text-red-800';
      case 'DELIVERING':
        return 'bg-blue-100 text-blue-800';
      default:
        return 'bg-gray-100 text-gray-800';
    }
  };

  // 타임스탬프 포맷팅
  const formatTimestamp = (timestamp) => {
    return new Date(timestamp).toLocaleString('ko-KR', {
      hour: '2-digit',
      minute: '2-digit',
      month: 'short',
      day: 'numeric',
    });
  };

  return (
    <div className="bg-white p-4 rounded-lg shadow">
      <h2 className="text-lg font-semibold text-gray-800 mb-4">최근 주문</h2>
      <div className="overflow-x-auto">
        <table className="min-w-full divide-y divide-gray-200">
          <thead className="bg-gray-50">
            <tr>
              <th className="px-4 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                테이블
              </th>
              <th className="px-4 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                시간
              </th>
              <th className="px-4 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                아이템
              </th>
              <th className="px-4 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                금액
              </th>
              <th className="px-4 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                상태
              </th>
              <th className="px-4 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                액션
              </th>
            </tr>
          </thead>
          <tbody className="bg-white divide-y divide-gray-200">
            {orders.map((order) => (
              <tr key={order.id} className="hover:bg-gray-50">
                <td className="px-4 py-3 whitespace-nowrap">
                  <span className="font-medium">테이블 {order.tableNumber}</span>
                </td>
                <td className="px-4 py-3 whitespace-nowrap text-sm text-gray-500">
                  {formatTimestamp(order.timestamp)}
                </td>
                <td className="px-4 py-3 whitespace-nowrap text-sm">
                  {order.itemCount}개 항목
                </td>
                <td className="px-4 py-3 whitespace-nowrap text-sm">
                  <div className="flex items-center">
                    <DollarSign size={14} className="mr-1 text-gray-400" />
                    {typeof order.totalPrice === 'number' 
                      ? new Intl.NumberFormat('ko-KR').format(order.totalPrice) 
                      : order.totalPrice}원
                  </div>
                </td>
                <td className="px-4 py-3 whitespace-nowrap">
                  <div className="flex items-center">
                    <div className="mr-2">{getStatusIcon(order.status)}</div>
                    <span className={`px-2 py-1 text-xs rounded-full ${getStatusBadgeClass(order.status)}`}>
                      {order.status}
                    </span>
                  </div>
                </td>
                <td className="px-4 py-3 whitespace-nowrap text-sm text-gray-500">
                  <button 
                    onClick={() => onViewOrder(order.id)}
                    className="text-blue-600 hover:text-blue-800 flex items-center"
                  >
                    <Eye size={16} className="mr-1" />
                    보기
                  </button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default RecentOrders; 