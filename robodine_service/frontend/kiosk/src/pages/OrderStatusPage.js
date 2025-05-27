import React, { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import { useNavigate } from 'react-router-dom';
import { useUnifiedWebSockets } from '../context/UnifiedWebSocketProvider';
import Sidebar from '../components/Layout/Sidebar';
import NotificationOverlay from '../components/Notifications/NotificationOverlay';
import { useLanguage } from '../context/LanguageContext';
import axios from 'axios';
import styled from 'styled-components';
import translations from '../locale/translations';
// import { useWebSocket } from '../context/WebSocketContext';

const Base_API_URL = process.env.REACT_APP_BASE_URL

// 메뉴 아이템 카드 컴포넌트
const MenuItemCard = ({ item, onCancelItem, remainingTime }) => {
  const timerRef = useRef(null);
  const { t, language } = useLanguage();

  // 상태에 따른 배경색 설정
  const getBgColor = (status) => {
    switch (status) {
      case 'cooking': return 'bg-[#FEF3C7] border-[#D69E2E]'; // 노란색 (준비중)
      case 'ready': return 'bg-[#DEF7EC] border-[#48BB78]'; // 초록색 (완료)
      default: return 'bg-[#EDF2F7] border-gray-300'; // 회색 (대기중)
    }
  };

  // 상태 텍스트 변환
  const getStatusText = (status) => {
    switch (status) {
      case 'cooking': return t('orderStatus.cooking');
      case 'ready': return t('orderStatus.ready');
      default: return t('orderStatus.waiting');
    }
  };

  const formattedPrice = new Intl.NumberFormat('ko-KR').format(item.price);

  // waiting, cooking 상태일 때만 취소 버튼 활성화
  const canCancel = item.status === 'waiting' || item.status === 'cooking';

  // 시간을 정수로 표시
  const formattedTime = Math.max(0, remainingTime).toFixed(0);

  // 메뉴 아이템 이름 다국어 지원
  const getMenuName = (name) => {
    if (language === 'ko') return name;
    
    // translations.js의 menuData에서 해당 메뉴의 번역된 이름 찾기
    const menuKeys = Object.keys(translations[language].menuData || {});
    const menuKey = menuKeys.find(key => key === name);
    
    if (menuKey) {
      return translations[language].menuData[menuKey].name;
    }
    
    return name;
  };
  
  // 가격 포맷
  const formatPrice = (price) => {
    return new Intl.NumberFormat(
      language === 'en' ? 'en-US' : language === 'ja' ? 'ja-JP' : 'ko-KR', 
      { 
        style: 'currency', 
        currency: language === 'en' ? 'USD' : language === 'ja' ? 'JPY' : 'KRW',
        minimumFractionDigits: 0 
      }
    ).format(price);
  };
  
  const statusText = getStatusText(item.status);
  const statusBgColor = getBgColor(item.status);

  return (
    <div className={`w-56 rounded-lg shadow-md border-l-4 flex-shrink-0 ${statusBgColor}`}>
      <div className="p-4 relative">
        <div className="flex justify-between items-center mb-2">
          <h3 className="text-xl font-bold truncate">{getMenuName(item.name)}</h3>
          <span className={`text-sm font-medium px-2 py-1 rounded-full whitespace-nowrap
            ${item.status === 'ready' ? 'text-green-800 bg-green-100' : 
              item.status === 'cooking' ? 'text-yellow-800 bg-yellow-100' : 
              'text-gray-600 bg-gray-200'}`}>
            {statusText}
          </span>
        </div>
        <div className="h-36 w-full mb-3 rounded overflow-hidden">
          <div className="w-full h-full relative">
            <img 
              src={item.image_url || `http://192.168.0.156:8000/images/menu/${item.name.toLowerCase()}.png`} 
              alt={getMenuName(item.name)} 
              className="w-full h-full object-contain bg-gray-100"
              onError={(e) => {
                e.target.src = 'http://192.168.0.156:8000/images/menu/default.png';
              }}
            />
          </div>
        </div>
        <div className="flex justify-between items-center mb-2">
          <span className="text-lg">{formatPrice(item.price)}</span>
          <span className="text-sm font-medium bg-white px-2 py-1 rounded-full">
            {item.qty}{t('orderStatus.items')}
          </span>
        </div>
        
        {canCancel && (
          <button 
            onClick={() => onCancelItem(item.menuItemId, item.orderId)}
            className="w-full bg-red-100 text-red-700 py-2 rounded-md font-medium hover:bg-red-200 transition-colors"
          >
            {t('orderStatus.cancel')}
          </button>
        )}
      </div>
    </div>
  );
};

// 취소 확인 모달 컴포넌트
const CancelConfirmModal = ({ isOpen, onClose, onConfirm, title, message, isProcessing }) => {
  const { t } = useLanguage();
  
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 z-50 flex items-center justify-center">
      <div className="bg-white rounded-lg p-6 max-w-md w-full shadow-xl">
        <h2 className="text-2xl font-bold mb-4">{title}</h2>
        <p className="mb-6 text-gray-700">{message}</p>
        <div className="flex justify-end space-x-4">
          <button
            onClick={onClose}
            disabled={isProcessing}
            className="px-4 py-2 bg-gray-200 text-gray-800 rounded-md hover:bg-gray-300"
          >
            {t('common.cancel')}
          </button>
          <button
            onClick={onConfirm}
            disabled={isProcessing}
            className={`px-4 py-2 rounded-md text-white ${
              isProcessing ? 'bg-gray-400 cursor-not-allowed' : 'bg-red-600 hover:bg-red-700'
            }`}
          >
            {isProcessing ? t('common.loading') : t('common.confirm')}
          </button>
        </div>
      </div>
    </div>
  );
};

const OrderStatusPage = () => {
  const navigate = useNavigate();
  // 통합 WebSocket 컨텍스트 사용
  const { data, connected, errors, currentCustomer, customerOrders, refreshTopic, orderItemsRemainingTime } = useUnifiedWebSockets();
  const { t, language } = useLanguage();
  
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [notifications, setNotifications] = useState([]);
  const [selectedCategory, setSelectedCategory] = useState('추천');
  const [isCancellingOrder, setIsCancellingOrder] = useState(false);
  const [cancelLoadingFlag, setCancelLoadingFlag] = useState(false);
  const prevOrderDataRef = useRef(null);
  const notificationTimeoutRef = useRef(null);
  
  // 모달 상태 관리
  const [cancelOrderModal, setCancelOrderModal] = useState({
    isOpen: false,
    itemId: null,
    orderId: null,
    isFullOrder: false
  });
  
  // 테이블 ID는 localStorage에서 가져오기
  const tableId = parseInt(localStorage.getItem('kioskTableId') || '1');

  // 알림 추가 함수 (중복 방지)
  const addNotification = (message) => {
    const id = Date.now();
    
    // 동일한 메시지가 있는지 확인
    const isDuplicate = notifications.some(n => n.message === message);
    if (isDuplicate) return;
    
    // 이전 타이머 취소
    if (notificationTimeoutRef.current) {
      clearTimeout(notificationTimeoutRef.current);
    }
    
    setNotifications(prev => [...prev, { id, message }]);
    
    // 5초 후 자동으로 알림 제거
    notificationTimeoutRef.current = setTimeout(() => {
      setNotifications(prev => prev.filter(n => n.id !== id));
    }, 5000);
  };

  // 알림 닫기 핸들러
  const handleCloseNotification = (id) => {
    setNotifications(notifications.filter(n => n.id !== id));
  };

  // 주문 취소 모달 열기
  const openCancelOrderModal = () => {
    if (!currentCustomer || !orderData || isCancellingOrder) return;
    
    setCancelOrderModal({
      isOpen: true,
      itemId: null,
      orderId: null,
      isFullOrder: true
    });
  };

  // 주문 항목 취소 모달 열기
  const openCancelOrderItemModal = (menuItemId, orderId) => {
    if (!currentCustomer || !orderId || isCancellingOrder) return;
    
    setCancelOrderModal({
      isOpen: true,
      itemId: menuItemId,
      orderId: orderId,
      isFullOrder: false
    });
  };

  // 모달 닫기
  const closeCancelModal = () => {
    setCancelOrderModal({
      ...cancelOrderModal,
      isOpen: false
    });
  };

  // 주문 취소 확인 처리
  const confirmCancelOrder = async () => {
    const { isFullOrder, itemId, orderId } = cancelOrderModal;
    
    try {
      setIsCancellingOrder(true);
      
      if (isFullOrder) {
        // 전체 주문 취소 처리
        await handleFullOrderCancel();
      } else if (itemId && orderId) {
        // 개별 항목 취소 처리
        await handleOrderItemCancel(itemId, orderId);
      }
      
      closeCancelModal();
      refreshTopic('orders');
    } catch (error) {
      console.error(t('orderStatus.error.cancel'), error);
      addNotification(t('common.error'));
    } finally {
      setIsCancellingOrder(false);
    }
  };

  // 전체 주문 취소 처리
  const handleFullOrderCancel = async () => {
    if (!currentCustomer || !orderData || isCancellingOrder) return;
    
    setIsCancellingOrder(true);
    
    try {
      // 취소 후 데이터가 갱신될 때까지 로딩 표시를 위한 플래그
      setCancelLoadingFlag(true);
      
      const response = await fetch(`${Base_API_URL}/orders/cancel_order/${currentCustomer}`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json'
        }
      });
      
      if (!response.ok) {
        throw new Error('주문 취소 실패');
      }
      
      // 성공 알림
      addNotification(t('orderStatus.success.itemCancelled'));
      
      // 백엔드에서 웹소켓으로 자동 업데이트가 전송되므로
      // 일정 시간 후 로딩 상태만 해제 (2초로 단축)
      setTimeout(() => {
        setCancelLoadingFlag(false);
      }, 2000);
    } catch (err) {
      console.error('주문 취소 중 오류:', err);
      addNotification('주문 취소 중 오류가 발생했습니다. 다시 시도해주세요.');
      setCancelLoadingFlag(false);
    } finally {
      setIsCancellingOrder(false);
    }
  };
  
  // 주문 항목 취소 처리
  const handleOrderItemCancel = async (menuItemId, orderId) => {
    if (!currentCustomer || !orderId || isCancellingOrder) return;
    
    try {
      setIsCancellingOrder(true);
      
      // 취소 후 데이터가 갱신될 때까지 로딩 표시를 위한 플래그
      setCancelLoadingFlag(true);
      
      await axios.put(
        `${Base_API_URL}/orders/${orderId}/items/${menuItemId}/status`,
        { status: "CANCELLED" },
      );
      
      // 성공 알림
      addNotification(t('orderStatus.success.itemCancelled'));
      
      // 백엔드에서 웹소켓으로 자동 업데이트가 전송되므로
      // 일정 시간 후 로딩 상태만 해제 (2초로 단축)
      setTimeout(() => {
        setCancelLoadingFlag(false);
      }, 1000);
    } catch (err) {
      console.error('주문 항목 취소 중 오류:', err);
      addNotification('주문 항목 취소 중 오류가 발생했습니다. 다시 시도해주세요.');
      setCancelLoadingFlag(false);
    } finally {
      setIsCancellingOrder(false);
    }
  };

  // 초기 로딩 처리
  useEffect(() => {
    // 취소 중인 경우 로딩 상태를 유지함
    if (cancelLoadingFlag) {
      setIsLoading(true);
      return;
    }

    // 연결이 되어 있고 주문 데이터가 있는 경우 로딩 상태 해제
    if (connected.orders && connected.tables && 
        data.orders && data.tables) {
      // 주문 데이터가 비어있지 않은지 확인
      if (data.orders.orderitems && data.orders.orderitems.length > 0) {
        setIsLoading(false);
      } else if (customerOrders && customerOrders.length > 0) {
        setIsLoading(false);
      }
    } else {
      // 연결이 안되어 있으면 토픽 리프레시
      if (!connected.orders) refreshTopic('orders');
      if (!connected.tables) refreshTopic('tables');
    }
  }, [connected.orders, connected.tables, data.orders, data.tables, customerOrders, refreshTopic, cancelLoadingFlag]);

  // 주문 정보 추출 및 처리
  const getOrderData = () => {
    // 취소 중이거나 로딩 중일 때는 이전 데이터 유지
    if (cancelLoadingFlag && prevOrderDataRef.current) {
      return prevOrderDataRef.current;
    }
    
    // 고객 ID가 없거나 고객 주문이 없으면 null 반환
    if (!currentCustomer || !customerOrders || customerOrders.length === 0) {
      return null;
    }

    try {
      // console.log('주문 데이터 처리 시작:', customerOrders);
      
      // 현재 고객의 모든 주문을 시간순으로 정렬 (최신 순)
      const sortedOrders = [...customerOrders].sort((a, b) => {
        const timestampA = new Date(a['Order.timestamp'] || a.timestamp);
        const timestampB = new Date(b['Order.timestamp'] || b.timestamp);
        return timestampB - timestampA;
      });
      
      // 가장 최근 주문
      const latestOrder = sortedOrders[0];

      // 취소되지 않은 주문만 필터링
      const activeOrders = sortedOrders.filter(order => 
        (order['Order.status'] || order.status) !== 'CANCELLED'
      );
      
      // 모든 주문 ID 수집 (취소된 주문 제외)
      const orderIds = activeOrders.map(order => order['Order.id'] || order.id);
      
      // 주문 상태 결정 로직
      // PREPARING 상태가 하나라도 있으면 PREPARING, 아니면 PLACED
      const hasPreparingOrder = sortedOrders.some(
        order => (order['Order.status'] || order.status) === 'PREPARING'
      );
      
      // 최종 주문 상태 결정
      let orderStatus;
      if (hasPreparingOrder) {
        orderStatus = 'PREPARING';
      } else {
        // 취소 상태는 무시하고, 대기 중(PLACED) 상태만 표시
        orderStatus = 'PLACED';
      }
      
      // 주문 시간 가져오기 (가장 최근 주문의 시간)
      const orderTimestamp = latestOrder['Order.timestamp'] || latestOrder.timestamp;
      
      // 결제 시간은 localStorage에서 가져오기 (OrderCompletePage에서 설정한 값)
      let paymentTimeStr = localStorage.getItem('lastPaymentTime');
      
      // 주문 아이템 가져오기
      const orderItems = [];
      if (data.orders && data.orders.orderitems && Array.isArray(data.orders.orderitems)) {
        // 모든 주문의 아이템들을 원본 주문 순서대로 수집
        const allItems = [];
        
        // 각 주문별로 아이템 수집
        sortedOrders.forEach(order => {
          const orderId = order['Order.id'] || order.id;
          const orderTimestamp = order['Order.timestamp'] || order.timestamp;
          
          // 현재 주문에 해당하는 아이템 필터링
          const itemsForOrder = data.orders.orderitems.filter(item => {
            const itemOrderId = item['OrderItem.order_id'] || item.order_id;
            return itemOrderId === orderId;
          });
          
          // 주문 정보를 포함하여 아이템 저장
          itemsForOrder.forEach(item => {
            allItems.push({
              ...item,
              orderTimestamp,
              orderId
            });
          });
        });
        
        // 각 메뉴 아이템 정보와 결합
        if (data.orders.menuitems && Array.isArray(data.orders.menuitems)) {
          // console.log('모든 아이템:', allItems);
          allItems.forEach(item => {
            const menuItemId = item['OrderItem.menu_item_id'] || item.menu_item_id;
            const quantity = item['OrderItem.quantity'] || item.quantity;
            const orderTimestamp = item.orderTimestamp;
            const orderId = item['OrderItem.order_id'] || item.id;

            // console.log('주문 아이템:', item);
            
            // 메뉴 아이템 찾기
            const menuItem = data.orders.menuitems.find(mi => 
              (mi['MenuItem.id'] || mi.id) === menuItemId
            );
            
            if (menuItem) {
              const name = menuItem['MenuItem.name'] || menuItem.name;
              const price = menuItem['MenuItem.price'] || menuItem.price;
              const image_url = menuItem['MenuItem.image_url'] || menuItem.image_url;
              
              // 개별 아이템 상태 결정 
              // 실제 데이터에서는 OrderItem.status를 사용해야 함
              let status = 'waiting';
              let eta = 0;
              
              // OrderItem 상태 확인
              const orderItemStatus = item['OrderItem.status'] || 'PLACED';
              // 1) 개별 아이템 상태 반영
              switch (orderItemStatus) {
                case 'CANCELLED':
                  status = 'cancelled';
                  break;
                case 'PREPARING':   // 백엔드에서 이렇게 내려온다면
                case 'COOKING':     // 혹은 COOKING
                  status = 'cooking';
                  eta = menuItem['MenuItem.prepare_time'] || 5;
                  break;
                case 'SERVED':
                case 'COMPLETED':
                  status = 'ready';
                  break;
                default:
                  status = 'waiting';
              }
              orderItems.push({
                id: `${orderId}-${menuItemId}`, // 고유 ID 생성
                orderId: orderId,
                menuItemId,
                name,
                qty: quantity,
                price,
                status,
                eta,
                orderTimestamp,
                image_url
              });
              // console.log('메뉴 아이템:', orderItems);
            }
          });
        }
      }

      // 취소되지 않은 메뉴 아이템만 필터링하여 총 개수 계산
      const totalMenuItems = orderItems
        .filter(item => item.status !== 'cancelled')
        .reduce((sum, item) => sum + item.qty, 0);

      // 총 금액 계산 (취소되지 않은 메뉴 아이템만 계산)
      const total = orderItems
        .filter(item => item.status !== 'cancelled')
        .reduce((sum, item) => sum + (item.price * item.qty), 0);

      // 주문 ID가 없으면 빈 문자열 반환
      const formattedOrderIds = orderIds.length > 0 ? orderIds.join(', #') : '';

      // 결제 시각 포맷팅
      const formattedPaymentTime = paymentTimeStr 
        ? formatDateTime(paymentTimeStr) 
        : '-';

      return {
        id: formattedOrderIds, // 모든 주문 ID를 콤마로 구분하여 표시 (취소된 주문 제외)
        tableName: `테이블 ${tableId}`,
        createdAt: formatDateTime(orderTimestamp),
        paidAt: formattedPaymentTime,
        paymentMethod: '신용카드', // 실제 구현에서는 결제 방법 정보 필요
        total,
        items: orderItems,
        status: orderStatus,
        orderCount: totalMenuItems
      };
    } catch (err) {
      console.error('주문 데이터 처리 중 오류:', err);
      setError('주문 데이터를 처리하는 중 오류가 발생했습니다.');
      return null;
    }
  };

  // 날짜 포맷팅 함수
  const formatDateTime = (dateStr) => {
    if (!dateStr) return '-';
    const date = new Date(dateStr);
    return date.toLocaleString(
      language === 'en' ? 'en-US' : language === 'ja' ? 'ja-JP' : 'ko-KR', 
      {
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        hour: '2-digit',
        minute: '2-digit',
        hour12: language === 'en' ? true : false
      }
    );
  };

  // 새로고침 처리
  const handleRefresh = () => {
    refreshTopic('orders');
    refreshTopic('tables');
    addNotification(t('orderStatus.refreshed'));
  };

  // 주문 상태 변경 감지 및 알림
  useEffect(() => {
    const orderData = getOrderData();
    
    // 취소 로딩 중이 아닐 때만 이전 주문 데이터 갱신
    if (!cancelLoadingFlag && orderData) {
      prevOrderDataRef.current = orderData;
    }
    
    const prevOrderData = prevOrderDataRef.current;
    
    if (orderData && prevOrderData) {
      // 주문 상태 변경 확인
      if (orderData.status !== prevOrderData.status) {
        const statusText = {
          'PENDING': t('orderStatus.waiting'),
          'PLACED': t('orderStatus.waiting'),
          'PREPARING': t('orderStatus.cooking'),
          'SERVING': t('orderStatus.cooking'),
          'SERVED': t('orderStatus.ready'),
          'COMPLETED': t('orderStatus.ready'),
          'CANCELLED': t('common.cancel')
        }[orderData.status] || orderData.status;
        
        addNotification(`주문 상태가 ${statusText}로 변경되었습니다.`);
      }
      
      // 각 메뉴 아이템 상태 변경 확인 (상태 변경된 항목만 1회 알림)
      const changedItems = new Set();
      
      orderData.items.forEach(item => {
        // 이전 데이터에서 같은 아이템 찾기
        const prevItem = prevOrderData.items.find(pi => pi.id === item.id);
        
        // 이전 아이템이 있고, 상태가 변경되었을 때 알림 (중복 알림 방지)
        if (prevItem && prevItem.status !== item.status && !changedItems.has(item.id)) {
          let statusMessage = '';
          
          switch(item.status) {
            case 'ready':
              statusMessage = `${item.name} 메뉴가 준비 완료되었습니다.`;
              break;
            case 'cooking':
              statusMessage = `${item.name} 메뉴가 조리 중입니다.`;
              break;
            case 'cancelled':
              statusMessage = `${item.name} 메뉴가 취소되었습니다.`;
              break;
            default:
              statusMessage = `${item.name} 메뉴의 상태가 변경되었습니다.`;
          }
          
          // 알림 추가 및 중복 방지 표시
          addNotification(statusMessage);
          changedItems.add(item.id);
        }
      });
    }
    
    // 컴포넌트 언마운트 시 타임아웃 정리
    return () => {
      if (notificationTimeoutRef.current) {
        clearTimeout(notificationTimeoutRef.current);
      }
    };
  }, [customerOrders, data.orders, cancelLoadingFlag]);

  // 현재 주문 상태
  const orderData = getOrderData();

  // 상태별로 아이템 정렬 (준비중 -> 대기중 -> 완료)
  const sortedMenuItems = orderData ? [...orderData.items]
    // CANCELLED 상태의 메뉴 항목 필터링
    .filter(item => item.status !== 'cancelled')
    .sort((a, b) => {
      // 상태에 따른 가중치
      const getWeight = (status) => {
        switch (status) {
          case 'cooking': return 0; // 준비중 (가장 앞)
          case 'waiting': return 1; // 대기중 (중간)
          case 'ready': return 2;   // 완료 (가장 뒤)
          default: return 3;
        }
      };
      
      // 상태 비교
      const weightA = getWeight(a.status);
      const weightB = getWeight(b.status);
      
      if (weightA !== weightB) {
        return weightA - weightB;
      }
      
      // 같은 상태는 주문 시간순으로 정렬 (오래된 순)
      return new Date(a.orderTimestamp) - new Date(b.orderTimestamp);
    }) : [];

  // 연결되지 않은 경우
  if ((!connected.orders || !connected.tables) && !isLoading) {
    return (
      <div className="flex h-screen bg-gray-100">
        <NotificationOverlay 
          notifications={notifications} 
          onClose={handleCloseNotification} 
        />
        <Sidebar 
          selectedCategory={selectedCategory}
          onSelectCategory={setSelectedCategory}
        />
        <div className="flex flex-col items-center justify-center p-6 h-full flex-grow">
          <div className="bg-red-100 text-red-700 p-4 rounded-md text-center w-full max-w-md">
            <p className="text-xl mb-4">서버 연결 오류</p>
            <p className="mb-4">주문 정보 서버에 연결할 수 없습니다. 네트워크 연결을 확인해주세요.</p>
            <button 
              onClick={handleRefresh}
              className="bg-red-600 text-white px-4 py-2 rounded-md"
            >
              다시 연결
            </button>
          </div>
        </div>
      </div>
    );
  }

  // 로딩 중인 경우
  if (isLoading || cancelLoadingFlag) {
    return (
      <div className="flex h-screen bg-gray-100">
        <NotificationOverlay 
          notifications={notifications} 
          onClose={handleCloseNotification} 
        />
        <Sidebar 
          selectedCategory={selectedCategory}
          onSelectCategory={setSelectedCategory}
        />
        <div className="flex flex-col items-center justify-center p-6 h-full flex-grow">
          <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-[#C49E69]"></div>
          {cancelLoadingFlag ? (
            <>
              <p className="mt-4 text-gray-700 text-xl font-medium">주문 취소 처리 완료</p>
              <p className="mt-2 text-gray-500">주문 정보가 업데이트 중입니다. 잠시만 기다려주세요.</p>
            </>
          ) : (
            <p className="mt-4 text-gray-600">주문 정보를 불러오는 중...</p>
          )}
        </div>
      </div>
    );
  }

  // 오류가 있는 경우
  if (error) {
    return (
      <div className="flex h-screen bg-gray-100">
        <NotificationOverlay 
          notifications={notifications} 
          onClose={handleCloseNotification} 
        />
        <Sidebar 
          selectedCategory={selectedCategory}
          onSelectCategory={setSelectedCategory}
        />
        <div className="flex flex-col items-center justify-center p-6 h-full flex-grow">
          <div className="bg-red-100 text-red-700 p-4 rounded-md text-center w-full max-w-md">
            <p className="text-xl mb-4">오류 발생</p>
            <p>{error}</p>
            <button 
              onClick={handleRefresh}
              className="mt-4 bg-red-600 text-white px-4 py-2 rounded-md"
            >
              다시 시도
            </button>
          </div>
        </div>
      </div>
    );
  }

  // 고객 정보가 없는 경우
  if (!currentCustomer) {
    return (
      <div className="flex h-screen bg-gray-100">
        <NotificationOverlay 
          notifications={notifications} 
          onClose={handleCloseNotification} 
        />
        <Sidebar 
          selectedCategory={selectedCategory}
          onSelectCategory={setSelectedCategory}
        />
        <div className="flex flex-col items-center justify-center p-6 h-full flex-grow">
          <div className="bg-blue-100 text-blue-700 p-6 rounded-md text-center w-full max-w-md">
            <p className="text-xl mb-4">고객 정보를 찾을 수 없습니다</p>
            <p className="mb-4">현재 테이블에 등록된 고객이 없습니다.</p>
            <button 
              onClick={() => navigate('/')}
              className="bg-blue-600 text-white px-4 py-2 rounded-md"
            >
              홈으로 돌아가기
            </button>
          </div>
        </div>
      </div>
    );
  }

  // 주문이 없는 경우
  if (!orderData) {
    return (
      <div className="flex h-screen bg-gray-100">
        <NotificationOverlay 
          notifications={notifications} 
          onClose={handleCloseNotification} 
        />
        <Sidebar 
          selectedCategory={selectedCategory}
          onSelectCategory={setSelectedCategory}
        />
        <div className="flex flex-col items-center justify-center p-6 h-full flex-grow">
          <div className="bg-yellow-100 text-yellow-800 p-6 rounded-md text-center w-full max-w-md">
            <p className="text-xl mb-4">진행 중인 주문이 없습니다</p>
            <p className="mb-4">테이블 {tableId}, 고객 ID: {currentCustomer}</p>
            <p>새로운 주문을 시작하려면 홈 화면으로 돌아가세요.</p>
            <button 
              onClick={() => navigate('/')}
              className="mt-4 bg-[#C49E69] text-white px-4 py-2 rounded-md"
            >
              메뉴 보러가기
            </button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="flex h-screen bg-gray-100">
      <NotificationOverlay 
        notifications={notifications} 
        onClose={handleCloseNotification} 
      />
      <Sidebar 
        selectedCategory={selectedCategory}
        onSelectCategory={setSelectedCategory}
      />
      
      {/* 취소 확인 모달 */}
      <CancelConfirmModal
        isOpen={cancelOrderModal.isOpen}
        onClose={closeCancelModal}
        onConfirm={confirmCancelOrder}
        title={cancelOrderModal.isFullOrder ? t('orderStatus.fullOrderCancel') : t('orderStatus.cancel')}
        message={cancelOrderModal.isFullOrder 
          ? t('orderStatus.confirmCancelAll') 
          : t('orderStatus.confirmCancelItem')}
        isProcessing={isCancellingOrder}
      />
      
      <div className="flex-grow overflow-auto p-6">
        {/* 페이지 헤더 */}
        <div className="flex justify-between items-center mb-6">
          <h1 className="text-3xl font-bold">{t('orderStatus.title')}</h1>
          <button 
            onClick={handleRefresh}
            className="bg-gray-200 text-gray-700 px-4 py-2 rounded-md"
          >
            {t('common.refresh')}
          </button>
        </div>

        {/* 주문 요약 카드 */}
        <div className="bg-white rounded-lg shadow p-6 mb-8">
          <div className="flex flex-wrap gap-4 items-center">
            <div className="flex-1 min-w-[200px]">
              <p className="text-xl mb-2">
                <strong>{t('orderStatus.orderNumber')}:</strong> #{orderData.id.startsWith('#') ? orderData.id.substring(1) : orderData.id}
              </p>
              <p className="text-xl mb-2"><strong>{t('orderStatus.table')}:</strong> {t('orderStatus.table')} {tableId}</p>
              <p className="text-xl mb-2"><strong>{t('orderStatus.customerId')}:</strong> {currentCustomer}</p>
            </div>
            <div className="flex-1 min-w-[200px]">
              <p className="text-xl mb-2"><strong>{t('orderStatus.title')}:</strong> 
                <span className={`ml-2 px-3 py-1 rounded-full text-base font-medium ${
                  orderData.status === 'SERVED' || orderData.status === 'COMPLETED' 
                    ? 'bg-green-100 text-green-800' 
                    : orderData.status === 'PREPARING' || orderData.status === 'SERVING'
                    ? 'bg-yellow-100 text-yellow-800'
                    : 'bg-blue-100 text-blue-800'
                }`}>
                  {
                    {
                      'PENDING': t('orderStatus.waiting'),
                      'PLACED': t('orderStatus.waiting'),
                      'PREPARING': t('orderStatus.cooking'),
                      'SERVING': t('orderStatus.cooking'),
                      'SERVED': t('orderStatus.ready'),
                      'COMPLETED': t('orderStatus.ready'),
                      'CANCELLED': t('common.cancel')
                    }[orderData.status] || orderData.status
                  }
                </span>
              </p>
              <p className="text-xl mb-2 text-blue-600"><strong>{t('orderStatus.menuTotalCount')}:</strong> {orderData.orderCount}{t('orderStatus.items')}</p>
              <p className="text-lg text-gray-500"><strong>{t('orderStatus.orderTime')}:</strong> {orderData.createdAt}</p>
            </div>
            
            {/* 주문 취소 버튼 (취소 가능한 상태일 때만 표시) */}
            {(orderData.status === 'PLACED' || orderData.status === 'PREPARING') && (
              <div className="flex items-center ml-4">
                <button
                  onClick={openCancelOrderModal}
                  disabled={isCancellingOrder}
                  className={`px-5 py-3 rounded-lg text-white text-lg font-bold ${
                    isCancellingOrder ? 'bg-gray-400 cursor-not-allowed' : 'bg-red-600 hover:bg-red-700'
                  }`}
                >
                  {isCancellingOrder ? t('common.loading') : t('orderStatus.fullOrderCancel')}
                </button>
              </div>
            )}
          </div>
        </div>

        {/* 메뉴별 준비 상태 (카드 형식) - 높이 최적화 */}
        <div className="bg-white rounded-lg shadow p-4 mb-4">
          <h2 className="text-xl font-medium mb-3">{t('orderStatus.menuPreparationStatus')}</h2>
          {sortedMenuItems.length > 0 ? (
            <div className="overflow-x-auto pb-2">
              <div className="flex space-x-3 min-w-max">
                {sortedMenuItems.map(item => (
                  <MenuItemCard 
                    key={item.id} 
                    item={item} 
                    onCancelItem={openCancelOrderItemModal}
                    remainingTime={orderItemsRemainingTime[item.itemId] || item.eta}
                  />
                ))}
              </div>
            </div>
          ) : (
            <p className="text-gray-500 text-center py-4">{t('cart.empty')}</p>
          )}
        </div>

        {/* 결제 내역 */}
        <div className="bg-white rounded-lg shadow p-6">
          <h2 className="text-2xl font-medium mb-4">{t('orderStatus.paymentDetails')}</h2>
          <div className="space-y-3 text-xl">
            <p><strong>{t('orderStatus.paymentMethod')}:</strong> {
              orderData.paymentMethod === '신용카드' ? t('orderStatus.creditCard') :
              orderData.paymentMethod === '현금' ? t('orderStatus.cash') :
              t('orderStatus.mobilePayment')
            }</p>
            <p><strong>{t('orderStatus.totalAmount')}:</strong> {new Intl.NumberFormat(
              language === 'en' ? 'en-US' : language === 'ja' ? 'ja-JP' : 'ko-KR', 
              { 
                style: 'currency', 
                currency: language === 'en' ? 'USD' : language === 'ja' ? 'JPY' : 'KRW',
                minimumFractionDigits: 0 
              }
            ).format(orderData.total)}</p>
            <p className="text-lg text-gray-500"><strong>{t('orderStatus.paymentTime')}:</strong> {orderData.paidAt || '-'}</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default OrderStatusPage; 