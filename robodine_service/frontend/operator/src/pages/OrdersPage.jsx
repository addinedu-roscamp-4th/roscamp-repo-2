import React, { useState, useEffect, useCallback } from 'react';
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
  AlertTriangle,
  XCircle,
  CheckCircle,
  Coffee,
  Truck
} from 'lucide-react';

const WS_BASE_URL = 'ws://127.0.0.1:8000/ws';

// 웹소켓 연결 함수
const useWebSocket = (topic, onMessageReceived) => {
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  
  useEffect(() => {
    const ws = new WebSocket(`${WS_BASE_URL}/${topic}`);
    
    ws.onopen = () => {
      console.log(`${topic} 웹소켓 연결됨`);
      setConnected(true);
      setError(null);
    };
    
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.type === 'update' && data.topic === topic) {
          onMessageReceived(data.data);
        }
      } catch (error) {
        console.error(`${topic} 웹소켓 메시지 처리 오류:`, error);
      }
    };
    
    ws.onclose = () => {
      console.log(`${topic} 웹소켓 연결 종료`);
      setConnected(false);
    };
    
    ws.onerror = (error) => {
      console.error(`${topic} 웹소켓 오류:`, error);
      setError(`연결 오류: ${error.message || '알 수 없는 오류'}`);
    };
    
    return () => {
      ws.close();
    };
  }, [topic, onMessageReceived]);
  
  return { connected, error };
};

const OrdersPage = () => {
  const [orders, setOrders] = useState([]);
  const [ordersDetailData, setOrdersDetailData] = useState({
    orders: [], kioskterminals: [], orderItems: [], menuitems: []
  });
  const [selectedOrder, setSelectedOrder] = useState(null);
  const [isDetailModalOpen, setIsDetailModalOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [searchTerm, setSearchTerm] = useState('');
  const [statusFilter, setStatusFilter] = useState('ALL');
  const { apiCall } = useAuth();

  // 주문 상세 정보 조회
  const fetchOrderDetails = useCallback(async (orderId) => {
    try {
      const data = await apiCall(`/api/orders/${orderId}`);
      return data;
    } catch (err) {
      console.error(`주문 ID ${orderId}의 상세 정보를 불러오는 중 오류가 발생했습니다:`, err);
      return null;
    }
  }, [apiCall]);

  // 초기 주문 데이터 로딩
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

  // 웹소켓을 통한 주문 데이터 실시간 업데이트
  const handleOrdersMessage = useCallback((data) => {
    console.log("주문 데이터 수신:", data);
    
    if (data && typeof data === 'object') {
      setOrdersDetailData({
        orders: data.orders || [],
        orderItems: data.orderitems || [],
        kioskterminals: data.kioskterminals || [],
        menuitems: data.menuitems || []
      });
      
      // 기존 주문 목록 업데이트
      if (Array.isArray(data.orders) && data.orders.length > 0) {
        // API에서 받은 형식으로 변환
        const updatedOrders = data.orders.flat().map(order => {
          return {
            id: order['Order.id'],
            customer_id: order['Order.customer_id'],
            robot_id: order['Order.robot_id'],
            table_id: order['Order.table_id'],
            status: order['Order.status'],
            timestamp: order['Order.timestamp'],
            served_at: order['Order.served_at']
          };
        });
        
        // 기존 주문 목록과 병합 (중복 제거)
        setOrders(prevOrders => {
          const orderMap = new Map();
          
          // 기존 주문 맵에 추가
          prevOrders.forEach(order => {
            orderMap.set(order.id, order);
          });
          
          // 새 주문 정보로 업데이트
          updatedOrders.forEach(order => {
            orderMap.set(order.id, order);
          });
          
          // 맵에서 다시 배열로 변환
          return Array.from(orderMap.values());
        });
      }
    }
  }, []);

  // 웹소켓 연결
  const ordersWS = useWebSocket('orders', handleOrdersMessage);

  // 주문 상태 업데이트 함수
  const updateOrderStatus = async (orderId, newStatus) => {
    try {
      await apiCall(`/api/orders/${orderId}/status`, 'PUT', { status: newStatus });
      
      // 상태 업데이트 후 주문 목록 갱신
      setOrders(prevOrders => 
        prevOrders.map(order => 
          order.id === orderId ? { ...order, status: newStatus } : order
        )
      );
      
      // 현재 선택된 주문이 있고, 해당 주문의 상태가 변경된 경우 선택된 주문도 업데이트
      if (selectedOrder && selectedOrder.id === orderId) {
        setSelectedOrder(prev => ({ ...prev, status: newStatus }));
      }
      
      return true;
    } catch (err) {
      console.error(`주문 상태 업데이트 중 오류가 발생했습니다:`, err);
      return false;
    }
  };

  // 주문 상세 정보 보기 함수
  const handleViewDetails = async (orderId) => {
    const orderDetails = await fetchOrderDetails(orderId);
    if (orderDetails) {
      setSelectedOrder(orderDetails);
      setIsDetailModalOpen(true);
    }
  };

  // 상태 표시 함수
  const getStatusBadge = (status) => {
    const statusConfig = {
      PENDING: { label: '대기중', color: 'bg-yellow-100 text-yellow-800 border-yellow-200', icon: <Clock size={14} className="mr-1" /> },
      PLACED: { label: '접수됨', color: 'bg-blue-100 text-blue-800 border-blue-200', icon: <CheckCircle size={14} className="mr-1" /> },
      PREPARING: { label: '준비중', color: 'bg-indigo-100 text-indigo-800 border-indigo-200', icon: <Coffee size={14} className="mr-1" /> },
      DELIVERING: { label: '배달중', color: 'bg-purple-100 text-purple-800 border-purple-200', icon: <Truck size={14} className="mr-1" /> },
      SERVED: { label: '완료', color: 'bg-green-100 text-green-800 border-green-200', icon: <Check size={14} className="mr-1" /> },
      CANCELLED: { label: '취소됨', color: 'bg-red-100 text-red-800 border-red-200', icon: <XCircle size={14} className="mr-1" /> },
    };

    const config = statusConfig[status] || { label: status, color: 'bg-gray-100 text-gray-800 border-gray-200' };

    return (
      <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium border ${config.color}`}>
        {config.icon}
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

  // 주문 데이터 처리 - 테이블 번호, 주문 항목 수, 총 가격 계산
  const processedOrders = orders.map(order => {
    // 주문 항목 정보 찾기
    const orderItems = ordersDetailData.orderItems.filter(item => 
      item['OrderItem.order_id'] === order.id
    );
    
    // 주문 항목의 총 수량과 가격 계산
    let totalQuantity = 0;
    let totalPrice = 0;
    
    orderItems.forEach(item => {
      const quantity = item['OrderItem.quantity'] || 0;
      totalQuantity += quantity;
      
      // 메뉴 항목 찾기
      const menuItem = ordersDetailData.menuitems.find(menu => 
        menu['MenuItem.id'] === item['OrderItem.menu_item_id']
      );
      
      if (menuItem) {
        totalPrice += quantity * (menuItem['MenuItem.price'] || 0);
      }
    });
    
    return {
      ...order,
      tableNumber: order.table_id,
      itemCount: totalQuantity,
      totalPrice: totalPrice
    };
  });

  // 주문 필터링
  const filteredOrders = processedOrders
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
                  <option value="PLACED">접수됨</option>
                  <option value="PREPARING">준비중</option>
                  <option value="DELIVERING">배달중</option>
                  <option value="SERVED">완료</option>
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
                        <button 
                          onClick={() => handleViewDetails(order.id)} 
                          className="text-blue-600 hover:text-blue-900 font-medium"
                        >
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

        {/* 주문 상세 정보 모달 */}
        {isDetailModalOpen && selectedOrder && (
          <div className="fixed inset-0 bg-gray-600 bg-opacity-50 flex items-center justify-center z-50">
            <div className="bg-white rounded-lg shadow-xl max-w-2xl w-full mx-4 max-h-[90vh] overflow-y-auto">
              {/* 모달 헤더 */}
              <div className="border-b px-6 py-4 flex items-center justify-between">
                <h2 className="text-xl font-semibold text-gray-800 flex items-center">
                  <ShoppingCart className="mr-2" size={20} />
                  주문 #{selectedOrder.id} 상세 정보
                </h2>
                <button 
                  onClick={() => setIsDetailModalOpen(false)}
                  className="text-gray-400 hover:text-gray-500"
                >
                  <X size={20} />
                </button>
              </div>
              
              {/* 모달 내용 */}
              <div className="px-6 py-4">
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mb-6">
                  <div>
                    <p className="text-sm text-gray-500 mb-1">주문 상태</p>
                    <p className="text-lg font-medium">{getStatusBadge(selectedOrder.status)}</p>
                  </div>
                  <div>
                    <p className="text-sm text-gray-500 mb-1">주문 시간</p>
                    <p className="text-lg font-medium">{formatDate(selectedOrder.timestamp)}</p>
                  </div>
                  <div>
                    <p className="text-sm text-gray-500 mb-1">테이블 번호</p>
                    <p className="text-lg font-medium">
                      {selectedOrder.table_id ? `테이블 ${selectedOrder.table_id}` : '없음'}
                    </p>
                  </div>
                  <div>
                    <p className="text-sm text-gray-500 mb-1">고객 ID</p>
                    <p className="text-lg font-medium">#{selectedOrder.customer_id}</p>
                  </div>
                  {selectedOrder.served_at && (
                    <div>
                      <p className="text-sm text-gray-500 mb-1">서빙 완료 시간</p>
                      <p className="text-lg font-medium">{formatDate(selectedOrder.served_at)}</p>
                    </div>
                  )}
                  {selectedOrder.robot_id && (
                    <div>
                      <p className="text-sm text-gray-500 mb-1">배정 로봇</p>
                      <p className="text-lg font-medium">로봇 #{selectedOrder.robot_id}</p>
                    </div>
                  )}
                </div>
                
                <div className="mb-6">
                  <h3 className="text-lg font-medium border-b pb-2 mb-3">주문 항목</h3>
                  <div className="bg-gray-50 rounded-lg p-4">
                    <table className="min-w-full divide-y divide-gray-200">
                      <thead>
                        <tr>
                          <th className="pb-2 text-left text-sm font-semibold text-gray-500">메뉴 ID</th>
                          <th className="pb-2 text-left text-sm font-semibold text-gray-500">수량</th>
                        </tr>
                      </thead>
                      <tbody className="divide-y divide-gray-200">
                        {selectedOrder.items.map((item, idx) => {
                          // 메뉴 항목 정보 찾기
                          const menuItem = ordersDetailData.menuitems.find(menu => 
                            menu['MenuItem.id'] === item.menu_item_id
                          );
                          
                          return (
                            <tr key={idx}>
                              <td className="py-2 text-sm">
                                {menuItem 
                                  ? `${menuItem['MenuItem.name']} (${menuItem['MenuItem.price']}원)` 
                                  : `메뉴 #${item.menu_item_id}`}
                              </td>
                              <td className="py-2 text-sm">{item.quantity}개</td>
                            </tr>
                          );
                        })}
                      </tbody>
                    </table>
                  </div>
                </div>
                
                {/* 상태 업데이트 버튼 */}
                <div className="border-t pt-4">
                  <h3 className="text-lg font-medium mb-3">주문 상태 업데이트</h3>
                  <div className="flex flex-wrap gap-2">
                    <button
                      onClick={() => updateOrderStatus(selectedOrder.id, 'PLACED')}
                      disabled={selectedOrder.status === 'PLACED'}
                      className="px-3 py-2 bg-blue-100 text-blue-800 rounded-md text-sm font-medium hover:bg-blue-200 disabled:opacity-50 disabled:cursor-not-allowed flex items-center"
                    >
                      <CheckCircle size={16} className="mr-1" />
                      접수됨
                    </button>
                    <button
                      onClick={() => updateOrderStatus(selectedOrder.id, 'PREPARING')}
                      disabled={selectedOrder.status === 'PREPARING'}
                      className="px-3 py-2 bg-indigo-100 text-indigo-800 rounded-md text-sm font-medium hover:bg-indigo-200 disabled:opacity-50 disabled:cursor-not-allowed flex items-center"
                    >
                      <Coffee size={16} className="mr-1" />
                      준비중
                    </button>
                    <button
                      onClick={() => updateOrderStatus(selectedOrder.id, 'DELIVERING')}
                      disabled={selectedOrder.status === 'DELIVERING'}
                      className="px-3 py-2 bg-purple-100 text-purple-800 rounded-md text-sm font-medium hover:bg-purple-200 disabled:opacity-50 disabled:cursor-not-allowed flex items-center"
                    >
                      <Truck size={16} className="mr-1" />
                      배달중
                    </button>
                    <button
                      onClick={() => updateOrderStatus(selectedOrder.id, 'SERVED')}
                      disabled={selectedOrder.status === 'SERVED'}
                      className="px-3 py-2 bg-green-100 text-green-800 rounded-md text-sm font-medium hover:bg-green-200 disabled:opacity-50 disabled:cursor-not-allowed flex items-center"
                    >
                      <Check size={16} className="mr-1" />
                      완료
                    </button>
                    <button
                      onClick={() => updateOrderStatus(selectedOrder.id, 'CANCELLED')}
                      disabled={selectedOrder.status === 'CANCELLED'}
                      className="px-3 py-2 bg-red-100 text-red-800 rounded-md text-sm font-medium hover:bg-red-200 disabled:opacity-50 disabled:cursor-not-allowed flex items-center"
                    >
                      <XCircle size={16} className="mr-1" />
                      취소
                    </button>
                  </div>
                </div>
              </div>
            </div>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default OrdersPage; 