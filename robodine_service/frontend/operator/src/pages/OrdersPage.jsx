import React, { useState, useEffect, useCallback, useMemo, useRef } from 'react';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useWebSockets } from '../contexts/WebSocketContext';
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
  Truck,
  DollarSign,
  RefreshCw
} from 'lucide-react';

const OrdersPage = () => {
  const [selectedOrder, setSelectedOrder] = useState(null);
  const [isDetailModalOpen, setIsDetailModalOpen] = useState(false);
  const [searchTerm, setSearchTerm] = useState('');
  const [statusFilter, setStatusFilter] = useState('ALL');
  const [selectedSort, setSelectedSort] = useState('RECENT'); // RECENT, STATUS, PRICE
  const [showSortMenu, setShowSortMenu] = useState(false);
  const { apiCall } = useAuth();
  
  // 현재 주문 데이터를 저장할 ref
  const ordersDataRef = useRef({
    orders: [],
    orderItems: [],
    menuitems: []
  });
  
  // 로딩 상태를 별도로 관리
  const [isManualLoading, setIsManualLoading] = useState(false);
  const [initialLoadDone, setInitialLoadDone] = useState(false);
  const [localError, setLocalError] = useState(null);
  
  // 웹소켓 컨텍스트에서 데이터 가져오기
  const { data, errors, connected, refreshTopic } = useWebSockets();
  
  const isLoading = !initialLoadDone || isManualLoading;
  const error = localError || errors.orders;

  // 정렬 옵션 라벨 맵핑
  const sortLabels = {
    RECENT: '최신순',
    STATUS: '상태순',
    PRICE: '금액순',
    TABLE: '테이블 번호순',
  };

  // 데이터 새로고침 시 현재 데이터와 비교하여 변경사항만 업데이트
  useEffect(() => {
    if (data.orders) {
      try {
        // console.log('WebSocket 데이터 구조 분석:', {
        //   orders: data.orders.orders ? `${data.orders.orders.length}개` : '없음',
        //   orderitems: data.orders.orderitems ? `${data.orders.orderitems.length}개` : '없음',
        //   menuitems: data.orders.menuitems ? `${data.orders.menuitems.length}개` : '없음'
        // });
  
        // 데이터가 비어있는지 확인
        const isEmpty = 
          (!data.orders.orders || data.orders.orders.length === 0) &&
          (!data.orders.orderitems || data.orders.orderitems.length === 0) &&
          (!data.orders.menuitems || data.orders.menuitems.length === 0);
          
        if (isEmpty && initialLoadDone) {
          console.log('수신된 데이터가 비어있습니다. 기존 데이터를 유지합니다.');
          setIsManualLoading(false);
          return;
        }
  
        // 데이터 구조 검증 후 일관된 형식으로 변환
        const newOrdersData = {
          orders: Array.isArray(data.orders.orders) ? data.orders.orders : [],
          orderItems: Array.isArray(data.orders.orderitems) ? data.orders.orderitems : [],
          menuitems: Array.isArray(data.orders.menuitems) ? data.orders.menuitems : []
        };
  
        // console.log('수신된 주문 데이터:', newOrdersData);
  
        // 첫 로드 시 데이터 저장
        if (!initialLoadDone) {
          // console.log('첫 로드: 데이터 전체 저장');
          ordersDataRef.current = newOrdersData;
          setInitialLoadDone(true);
          setIsManualLoading(false);
          setLocalError(null); // 첫 로드 성공 시 에러 초기화
          return;
        }
  
        // 데이터 병합
        // 주문 데이터 병합
        if (newOrdersData.orders.length > 0) {
          newOrdersData.orders = newOrdersData.orders[0]
          // console.log(`주문 데이터 ${newOrdersData.orders.length}개 병합`);
          
          const existingOrdersMap = new Map();
          ordersDataRef.current.orders.forEach(order => {
            const orderId = order['Order.id'] || order.id;
            if (orderId) {
              existingOrdersMap.set(orderId.toString(), order);
            }
          });
  
          newOrdersData.orders.forEach(order => {
            const orderId = order['Order.id'] || order.id;
            if (orderId) {
              existingOrdersMap.set(orderId.toString(), order);
            }
          });
  
          ordersDataRef.current.orders = Array.from(existingOrdersMap.values());
          // console.log(`병합 후 주문 개수: ${ordersDataRef.current.orders.length}`);
        }
  
        // 주문 항목 데이터 병합
        if (newOrdersData.orderItems.length > 0) {
          // console.log(`주문 항목 데이터 ${newOrdersData.orderItems.length}개 병합`);
          const existingOrderItemsMap = new Map();
  
          ordersDataRef.current.orderItems.forEach(item => {
            const itemId = item['OrderItem.id'] || item.id;
            const orderId = item['OrderItem.order_id'] || item.order_id;
            if (itemId) {
              existingOrderItemsMap.set(`${orderId}-${itemId}`, item);
            } else if (orderId) {
              // ID가 없으면 주문ID-메뉴ID 조합으로 유일키 생성
              const menuItemId = item['OrderItem.menu_item_id'] || item.menu_item_id;
              existingOrderItemsMap.set(`${orderId}-${menuItemId}`, item);
            }
          });
  
          newOrdersData.orderItems.forEach(item => {
            const itemId = item['OrderItem.id'] || item.id;
            const orderId = item['OrderItem.order_id'] || item.order_id;
            if (itemId) {
              existingOrderItemsMap.set(`${orderId}-${itemId}`, item);
            } else if (orderId) {
              // ID가 없으면 주문ID-메뉴ID 조합으로 유일키 생성
              const menuItemId = item['OrderItem.menu_item_id'] || item.menu_item_id;
              existingOrderItemsMap.set(`${orderId}-${menuItemId}`, item);
            }
          });
  
          ordersDataRef.current.orderItems = Array.from(existingOrderItemsMap.values());
        }
  
        // 메뉴 항목 데이터 병합
        if (newOrdersData.menuitems.length > 0) {
          // console.log(`메뉴 항목 데이터 ${newOrdersData.menuitems.length}개 병합`);
          const existingMenuItemsMap = new Map();
  
          ordersDataRef.current.menuitems.forEach(item => {
            const menuId = item['MenuItem.id'] || item.id;
            if (menuId !== undefined) {
              existingMenuItemsMap.set(menuId.toString(), item);
            }
          });
  
          newOrdersData.menuitems.forEach(item => {
            const menuId = item['MenuItem.id'] || item.id;
            if (menuId !== undefined) {
              existingMenuItemsMap.set(menuId.toString(), item);
            }
          });
  
          ordersDataRef.current.menuitems = Array.from(existingMenuItemsMap.values());
        }
  
        // console.log('데이터 병합 완료');
        // console.log('현재 저장된 데이터:', {
        //   orders: ordersDataRef.current.orders.length,
        //   orderItems: ordersDataRef.current.orderItems.length,
        //   menuitems: ordersDataRef.current.menuitems.length
        // });
        
        setLocalError(null); // 데이터 로드 성공 시 에러 초기화
      } catch (error) {
        console.error('데이터 처리 중 오류 발생:', error);
        setLocalError('데이터 처리 중 오류가 발생했습니다.');
      } finally {
        setIsManualLoading(false);
      }
    }
  }, [data.orders, initialLoadDone]);
  

  // 주문 데이터 처리 - 현재 저장된 데이터 기반
  const processedOrders = useMemo(() => {
    const rawOrders = ordersDataRef.current.orders || [];
    // console.log(`주문 데이터 처리 시작 - ${rawOrders.length}개 주문 처리 중`);
    
    const processed = rawOrders.map(o => {
      const orderId = o['Order.id'] || o.id;
      const status = o['Order.status'] || o.status;
      const tableId = o['Order.table_id'] || o.table_id;
      const customerId = o['Order.customer_id'] || o.customer_id;
      const robotId = o['Order.robot_id'] || o.robot_id;
      const servedAt = o['Order.served_at'] || o.served_at;
      const tsRaw = o['Order.timestamp'] || o.timestamp || '';
      const timestamp = tsRaw.split('.')[0];
  
      // 주문 항목 찾기 - 객체 키 접근 방식 개선
      const orderItems = ordersDataRef.current.orderItems
        ? ordersDataRef.current.orderItems.filter(item => {
            const itemOrderId = item['OrderItem.order_id'] !== undefined 
              ? item['OrderItem.order_id'] 
              : (item.order_id !== undefined ? item.order_id : null);
            return itemOrderId === orderId;
          })
        : [];
  
      // 주문 항목의 총 수량과 가격 계산
      let totalQuantity = 0;
      let totalPrice = 0;
      const items = [];
      
      orderItems.forEach(item => {
        const itemId = item['OrderItem.id'] !== undefined ? item['OrderItem.id'] : (item.id !== undefined ? item.id : null);
        const menuItemId = item['OrderItem.menu_item_id'] !== undefined 
          ? item['OrderItem.menu_item_id'] 
          : (item.menu_item_id !== undefined ? item.menu_item_id : null);
        const quantity = item['OrderItem.quantity'] !== undefined 
          ? item['OrderItem.quantity'] 
          : (item.quantity !== undefined ? item.quantity : 0);
          
        totalQuantity += quantity;
  
        const menuItem = ordersDataRef.current.menuitems
          ? ordersDataRef.current.menuitems.find(menu => {
              const menuId = menu['MenuItem.id'] !== undefined 
                ? menu['MenuItem.id'] 
                : (menu.id !== undefined ? menu.id : null);
              return menuId === menuItemId;
            })
          : null;
  
        if (menuItem) {
          const price = menuItem['MenuItem.price'] !== undefined 
            ? menuItem['MenuItem.price'] 
            : (menuItem.price !== undefined ? menuItem.price : 0);
          const name = menuItem['MenuItem.name'] !== undefined 
            ? menuItem['MenuItem.name'] 
            : (menuItem.name !== undefined ? menuItem.name : `메뉴 #${menuItemId}`);
          totalPrice += quantity * price;
  
          items.push({
            id: itemId,
            menu_item_id: menuItemId,
            name: name,
            quantity: quantity,
            price: price
          });
        }
      });
  
      return {
        id: orderId, 
        status: status,
        table: tableId, 
        tableNumber: tableId,
        table_id: tableId,
        customer_id: customerId,
        robot_id: robotId,
        served_at: servedAt,
        timestamp,
        items,
        itemCount: totalQuantity,
        totalPrice,
        totalQuantity
      };
    });
  
    // console.log(`주문 데이터 처리 완료 - ${processed.length}개 주문`);
    return processed;
  }, [ordersDataRef.current.orders, ordersDataRef.current.orderItems, ordersDataRef.current.menuitems]);
  

  // 최초 접속 시 데이터 로드
  useEffect(() => {
    if (!initialLoadDone && connected.orders) {
      refreshTopic('orders');
    }
    
    // 1분마다 자동 새로고침 (더 긴 간격)
    const refreshInterval = setInterval(() => {
      refreshTopic('orders');
    }, 60000);
    
    return () => clearInterval(refreshInterval);
  }, [refreshTopic, connected.orders, initialLoadDone]);

  // 수동 새로고침 핸들러
  const handleRefreshData = useCallback(() => {
    setIsManualLoading(true);
    refreshTopic('orders');
  }, [refreshTopic]);

  // 주문 상태 업데이트 함수
  const updateOrderStatus = async (orderId, newStatus) => {
    try {
      setIsManualLoading(true);
      // console.log(`주문 상태 업데이트 요청: ID=${orderId}, 상태=${newStatus}`);
      
      // API 경로를 수정하여 직접 /api/orders 경로 사용 (로깅 목적으로 수정)
      const response = await apiCall(`/api/orders/${orderId}/status`, 'PUT', { status: newStatus });
      // console.log('주문 상태 업데이트 응답:', response);
      
      // 선택된 주문도 업데이트
      if (selectedOrder && selectedOrder.id === orderId) {
        setSelectedOrder({...selectedOrder, status: newStatus});
      }
      
      // 상태 업데이트 후 주문 목록 새로고침
      refreshTopic('orders');
      return true;
    } catch (err) {
      console.error(`주문 상태 업데이트 중 오류가 발생했습니다:`, err);
      setIsManualLoading(false);
      setLocalError(`주문 상태 업데이트 중 오류가 발생했습니다. (${err.message || '알 수 없는 오류'})`);
      return false;
    }
  };

  // 주문 상세 정보 보기 함수
  const handleViewDetails = (orderId) => {
    const orderDetails = processedOrders.find(order => order.id === orderId);
    if (orderDetails) {
      setSelectedOrder(orderDetails);
      setIsDetailModalOpen(true);
    }
  };

  // 상태 표시 함수
  const getStatusBadge = (status) => {
    const statusLabels = {
      PENDING: '대기중',
      PLACED: '접수됨',
      PREPARING: '준비중',
      SERVING: '서빙중',
      SERVED: '완료',
      COMPLETED: '완료',
      CANCELLED: '취소됨'
    };
    
    const statusConfig = {
      PENDING: { label: statusLabels[status] || '대기중', color: 'bg-yellow-100 text-yellow-800 border-yellow-200', icon: <Clock size={14} className="mr-1" /> },
      PLACED: { label: statusLabels[status] || '접수됨', color: 'bg-blue-100 text-blue-800 border-blue-200', icon: <CheckCircle size={14} className="mr-1" /> },
      PREPARING: { label: statusLabels[status] || '준비중', color: 'bg-indigo-100 text-indigo-800 border-indigo-200', icon: <Coffee size={14} className="mr-1" /> },
      SERVING: { label: statusLabels[status] || '서빙중', color: 'bg-purple-100 text-purple-800 border-purple-200', icon: <Truck size={14} className="mr-1" /> },
      SERVED: { label: statusLabels[status] || '완료', color: 'bg-green-100 text-green-800 border-green-200', icon: <Check size={14} className="mr-1" /> },
      COMPLETED: { label: statusLabels[status] || '완료', color: 'bg-green-100 text-green-800 border-green-200', icon: <Check size={14} className="mr-1" /> },
      CANCELLED: { label: statusLabels[status] || '취소됨', color: 'bg-red-100 text-red-800 border-red-200', icon: <XCircle size={14} className="mr-1" /> },
    };

    const config = statusConfig[status] || { label: statusLabels[status] || status, color: 'bg-gray-100 text-gray-800 border-gray-200' };

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
    return new Intl.DateTimeFormat('ko-KR', {
      month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit', hour12: false
    }).format(date);
  };

  // 정렬된 주문 리스트 계산
  const sortedOrders = useMemo(() => {
    if (!processedOrders || processedOrders.length === 0) return [];
    
    const filtered = processedOrders.filter(order => 
      (statusFilter === 'ALL' || order.status === statusFilter) &&
      (searchTerm === '' || 
        order.id.toString().includes(searchTerm) || 
        (order.tableNumber && order.tableNumber.toString().includes(searchTerm)))
    );
    
    const copy = [...filtered];
    
    switch (selectedSort) {
      case 'STATUS':
        return copy.sort((a, b) => {
          // PREPARING 상태 우선
          if (a.status === 'PREPARING' && b.status !== 'PREPARING') return -1;
          if (a.status !== 'PREPARING' && b.status === 'PREPARING') return 1;
          // PLACED 상태 그 다음
          if (a.status === 'PLACED' && b.status !== 'PLACED') return -1;
          if (a.status !== 'PLACED' && b.status === 'PLACED') return 1;
          // 이후 시간순
          return new Date(b.timestamp) - new Date(a.timestamp);
        });
      case 'PRICE':
        return copy.sort((a, b) => b.totalPrice - a.totalPrice);
      case 'TABLE':
        return copy.sort((a, b) => {
          if (!a.tableNumber) return 1;
          if (!b.tableNumber) return -1;
          return a.tableNumber - b.tableNumber;
        });
      case 'RECENT':
      default:
        return copy.sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp));
    }
  }, [processedOrders, statusFilter, searchTerm, selectedSort]);

  const formatPrice = (price) => {
    return new Intl.NumberFormat('ko-KR', {
      style: 'currency',
      currency: 'KRW',
      minimumFractionDigits: 0
    }).format(price);
  };

  return (
    <Layout>
      <div className="container mx-auto p-4 sm:p-6">
        <div className="mb-6 flex justify-between items-center">
          <div>
            <h1 className="text-2xl font-bold text-gray-800 flex items-center">
              <ShoppingCart className="mr-2" />
              주문 관리
            </h1>
            <p className="text-gray-600">모든 주문 정보를 확인하고 관리합니다.</p>
          </div>
          <button
            onClick={handleRefreshData}
            className="p-2 bg-white border border-gray-300 rounded-md hover:bg-gray-50 flex items-center gap-2"
            disabled={isLoading}
          >
            <RefreshCw size={16} className={isLoading ? "animate-spin" : ""} />
            {isLoading ? "로딩 중" : "새로고침"}
          </button>
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
                  <option value="SERVED">완료</option>
                  <option value="CANCELLED">취소됨</option>
                </select>
                <Filter className="absolute left-3 top-2.5 text-gray-400" size={18} />
                <ChevronDown className="absolute right-3 top-2.5 text-gray-400" size={18} />
              </div>
            </div>
            
            <div className="w-full md:w-1/3">
              <label className="text-sm font-medium text-gray-700 mb-1 block">정렬 순서</label>
              <div className="relative">
                <button
                  className="flex items-center justify-between pl-10 pr-4 py-2 border border-gray-300 rounded-md w-full hover:bg-gray-50"
                  onClick={() => setShowSortMenu(!showSortMenu)}
                >
                  {sortLabels[selectedSort]}
                  <ChevronDown size={16} className="ml-1" />
                </button>
                <DollarSign className="absolute left-3 top-2.5 text-gray-400" size={18} />
                
                {showSortMenu && (
                  <div className="absolute left-0 right-0 top-full mt-1 bg-white rounded-md shadow-lg z-20 border border-gray-200">
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
              <span>{error} (웹소켓을 통해 데이터를 불러옵니다)</span>
            </div>
          </div>
        ) : sortedOrders.length === 0 ? (
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
                  {sortedOrders.map((order) => (
                    <tr key={order.id} className="hover:bg-gray-50">
                      <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">
                        #{order.id}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {order.table ? `테이블 ${order.table}` : '-'}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {formatDate(order.timestamp)}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {order.itemCount || 0}개
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {order.totalPrice 
                          ? formatPrice(order.totalPrice) 
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
                  {selectedOrder.items && selectedOrder.items.length > 0 ? (
                    <div className="bg-gray-50 rounded-lg p-4">
                      <table className="min-w-full divide-y divide-gray-200">
                        <thead>
                          <tr>
                            <th className="pb-2 text-left text-sm font-semibold text-gray-500">메뉴 이름</th>
                            <th className="pb-2 text-center text-sm font-semibold text-gray-500">수량</th>
                            <th className="pb-2 text-right text-sm font-semibold text-gray-500">가격</th>
                          </tr>
                        </thead>
                        <tbody className="divide-y divide-gray-200">
                          {selectedOrder.items.map((item, idx) => (
                            <tr key={idx}>
                              <td className="py-2 text-sm">{item.name}</td>
                              <td className="py-2 text-sm text-center">{item.quantity}개</td>
                              <td className="py-2 text-sm text-right">{formatPrice(item.price * item.quantity)}</td>
                            </tr>
                          ))}
                          
                          {/* 총계 행 */}
                          <tr className="border-t border-gray-300">
                            <td className="py-2 text-sm font-bold">총계</td>
                            <td className="py-2 text-sm font-bold text-center">
                              {selectedOrder.totalQuantity}개
                            </td>
                            <td className="py-2 text-sm font-bold text-right">
                              {formatPrice(selectedOrder.totalPrice)}
                            </td>
                          </tr>
                        </tbody>
                      </table>
                    </div>
                  ) : (
                    <div className="bg-gray-50 rounded-lg p-4 text-center text-gray-500">
                      주문 항목 정보가 없습니다.
                    </div>
                  )}
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