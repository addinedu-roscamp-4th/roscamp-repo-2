import React, { useState, useEffect } from 'react';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';
import { 
  ShoppingCart, 
  Search, 
  Filter, 
  Check, 
  X, 
  ChevronDown, 
  Clock,
  AlertTriangle
} from 'react-feather';

const OrdersPage = () => {
  const [orders, setOrders] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [searchTerm, setSearchTerm] = useState('');
  const [statusFilter, setStatusFilter] = useState('ALL');
  const { apiCall } = useAuth();

  useEffect(() => {
    const fetchOrders = async () => {
      setIsLoading(true);
      setError(null);

      try {
        const data = await apiCall('/api/orders');
        setOrders(data);
      } catch (err) {
        console.error('주문 정보를 불러오는 중 오류가 발생했습니다:', err);
        setError('주문 정보를 불러올 수 없습니다.');
      } finally {
        setIsLoading(false);
      }
    };

    fetchOrders();
  }, [apiCall]);

  // 상태 표시 함수
  const getStatusBadge = (status) => {
    const statusConfig = {
      PENDING: { label: '대기중', color: 'bg-yellow-100 text-yellow-800 border-yellow-200' },
      PREPARING: { label: '준비중', color: 'bg-blue-100 text-blue-800 border-blue-200' },
      DELIVERING: { label: '배달중', color: 'bg-purple-100 text-purple-800 border-purple-200' },
      COMPLETED: { label: '완료', color: 'bg-green-100 text-green-800 border-green-200' },
      CANCELLED: { label: '취소됨', color: 'bg-red-100 text-red-800 border-red-200' },
    };

    const config = statusConfig[status] || { label: status, color: 'bg-gray-100 text-gray-800 border-gray-200' };

    return (
      <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium border ${config.color}`}>
        {config.label}
      </span>
    );
  };

  // 날짜 포맷팅
  const formatDate = (dateString) => {
    if (!dateString) return '';
    const date = new Date(dateString);
    return date.toLocaleString('ko-KR', {
      year: 'numeric',
      month: 'long',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit'
    });
  };

  // 주문 필터링
  const filteredOrders = orders
    .filter(order => 
      (statusFilter === 'ALL' || order.status === statusFilter) &&
      (searchTerm === '' || 
        order.id.toString().includes(searchTerm) || 
        (order.tableNumber && order.tableNumber.toString().includes(searchTerm)))
    )
    .sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp));

  return (
    <Layout>
      <div className="container mx-auto p-4 sm:p-6">
        <div className="mb-6">
          <h1 className="text-2xl font-bold text-gray-800 flex items-center">
            <ShoppingCart className="mr-2" />
            주문 관리
          </h1>
          <p className="text-gray-600">모든 주문 정보를 확인하고 관리합니다.</p>
        </div>

        {/* 필터 및 검색 */}
        <div className="bg-white p-4 rounded-lg shadow mb-6">
          <div className="flex flex-col md:flex-row md:items-center justify-between space-y-3 md:space-y-0 md:space-x-4">
            <div className="w-full md:w-1/3">
              <label className="text-sm font-medium text-gray-700 mb-1 block">주문 검색</label>
              <div className="relative">
                <input
                  type="text"
                  placeholder="주문번호 또는 테이블 번호"
                  className="pl-10 pr-4 py-2 border border-gray-300 rounded-md w-full focus:ring-blue-500 focus:border-blue-500"
                  value={searchTerm}
                  onChange={(e) => setSearchTerm(e.target.value)}
                />
                <Search className="absolute left-3 top-2.5 text-gray-400" size={18} />
              </div>
            </div>
            
            <div className="w-full md:w-1/3">
              <label className="text-sm font-medium text-gray-700 mb-1 block">상태 필터</label>
              <div className="relative">
                <select
                  className="appearance-none pl-10 pr-8 py-2 border border-gray-300 rounded-md w-full focus:ring-blue-500 focus:border-blue-500"
                  value={statusFilter}
                  onChange={(e) => setStatusFilter(e.target.value)}
                >
                  <option value="ALL">모든 상태</option>
                  <option value="PENDING">대기중</option>
                  <option value="PREPARING">준비중</option>
                  <option value="DELIVERING">배달중</option>
                  <option value="COMPLETED">완료</option>
                  <option value="CANCELLED">취소됨</option>
                </select>
                <Filter className="absolute left-3 top-2.5 text-gray-400" size={18} />
                <ChevronDown className="absolute right-3 top-2.5 text-gray-400" size={18} />
              </div>
            </div>
          </div>
        </div>

        {/* 주문 목록 */}
        {isLoading ? (
          <div className="bg-white rounded-lg shadow p-6 text-center">
            <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500 mx-auto"></div>
            <p className="mt-4 text-gray-600">주문 정보를 불러오는 중...</p>
          </div>
        ) : error ? (
          <div className="bg-white rounded-lg shadow p-6">
            <div className="bg-red-50 text-red-700 p-4 rounded-md flex items-center">
              <AlertTriangle className="mr-2" size={20} />
              <span>{error}</span>
            </div>
          </div>
        ) : filteredOrders.length === 0 ? (
          <div className="bg-white rounded-lg shadow p-6 text-center">
            <ShoppingCart className="mx-auto h-12 w-12 text-gray-400 mb-4" />
            <h3 className="text-lg font-medium text-gray-900">주문 내역이 없습니다</h3>
            <p className="mt-1 text-gray-500">
              {searchTerm || statusFilter !== 'ALL' 
                ? '검색 조건에 맞는 주문을 찾을 수 없습니다.' 
                : '아직 등록된 주문이 없습니다.'}
            </p>
          </div>
        ) : (
          <div className="bg-white rounded-lg shadow overflow-hidden">
            <div className="overflow-x-auto">
              <table className="min-w-full divide-y divide-gray-200">
                <thead className="bg-gray-50">
                  <tr>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      주문번호
                    </th>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      테이블
                    </th>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      시간
                    </th>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      항목 수
                    </th>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      금액
                    </th>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      상태
                    </th>
                    <th scope="col" className="px-6 py-3 text-right text-xs font-medium text-gray-500 uppercase tracking-wider">
                      작업
                    </th>
                  </tr>
                </thead>
                <tbody className="bg-white divide-y divide-gray-200">
                  {filteredOrders.map((order) => (
                    <tr key={order.id} className="hover:bg-gray-50">
                      <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">
                        #{order.id}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {order.tableNumber ? `테이블 ${order.tableNumber}` : '-'}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {formatDate(order.timestamp)}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {order.itemCount || '-'}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {order.totalPrice 
                          ? `${order.totalPrice.toLocaleString()}원` 
                          : '-'}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap">
                        {getStatusBadge(order.status)}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-right text-sm space-x-2">
                        <button className="text-blue-600 hover:text-blue-900 font-medium">
                          상세보기
                        </button>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default OrdersPage; 