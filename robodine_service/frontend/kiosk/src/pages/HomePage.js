// HomePage.js
import React, { useState, useEffect, useRef } from 'react';
import { Link } from 'react-router-dom';
import { useCart } from '../context/CartContext';
import { useUnifiedWebSockets } from '../context/UnifiedWebSocketProvider';
import CartSidebar from '../components/Cart/CartSidebar';
import MenuGrid from '../components/Menu/MenuGrid';
import Sidebar from '../components/Layout/Sidebar';
import NotificationOverlay from '../components/Notifications/NotificationOverlay';

// 카테고리 섹션 헤더 컴포넌트
const CategoryHeader = ({ category, id }) => (
  <div id={`category-${id}`} className="pt-6 pb-4 px-8 border-b-4 border-[#C49E69] bg-[#F7F3EE] mb-4">
    <h2 className="text-3xl font-bold">{category}</h2>
  </div>
);

const HomePage = () => {
  // 웹소켓 컨텍스트 사용
  const { data, connected, refreshTopic } = useUnifiedWebSockets();
  
  // 장바구니 컨텍스트
  const { addToCart, cartItems, getTotalAmount, removeFromCart, increaseQuantity, decreaseQuantity } = useCart();
  
  // 상태 관리
  const [menuItems, setMenuItems] = useState([]);
  const [menuByCategory, setMenuByCategory] = useState({});
  const [categories, setCategories] = useState(['추천', '음식', '음료']);
  const [selectedCategory, setSelectedCategory] = useState('추천');
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [notifications, setNotifications] = useState([]);
  const [visibleCategory, setVisibleCategory] = useState('추천');
  const [isScrolling, setIsScrolling] = useState(false); // 스크롤 중인지 추적

  // 카테고리 참조
  const categoryRefs = useRef({});
  // 컨텐츠 영역 참조
  const contentRef = useRef(null);
  // 스크롤 타이머 참조
  const scrollTimerRef = useRef(null);
  // 메뉴 데이터 요청 플래그 추적
  const hasRequestedMenu = useRef(false);

  // 카테고리 변경시 스크롤 처리
  const scrollToCategory = (category) => {
    if (categoryRefs.current[category] && contentRef.current) {
      // console.log(`카테고리 ${category}로 스크롤 시작`);
      
      // 스크롤 중임을 표시
      setIsScrolling(true);
      
      // 현재 처리 중인 스크롤 타이머가 있으면 취소
      if (scrollTimerRef.current) {
        clearTimeout(scrollTimerRef.current);
        scrollTimerRef.current = null;
      }
      
      // 스크롤 위치 계산
      const headerOffset = 10;
      const elementPosition = categoryRefs.current[category].offsetTop;
      const offsetPosition = elementPosition - headerOffset;
      
      // 현재 스크롤 위치와 목표 위치의 차이가 작으면 즉시 이동 (스크롤 애니메이션 없이)
      const currentPosition = contentRef.current.scrollTop;
      const distance = Math.abs(currentPosition - offsetPosition);
      
      if (distance < 50) {
        contentRef.current.scrollTop = offsetPosition;
        setIsScrolling(false);
        return;
      }
      
      // 스크롤 이벤트 제어를 위한 클린업 함수
      const cleanup = () => {
        setIsScrolling(false);
        if (contentRef.current) {
          contentRef.current.removeEventListener('scroll', scrollHandler);
        }
      };
      
      // 스크롤 중지 감지 함수
      const scrollHandler = () => {
        if (scrollTimerRef.current) {
          clearTimeout(scrollTimerRef.current);
        }
        
        scrollTimerRef.current = setTimeout(() => {
          cleanup();
          // console.log(`카테고리 ${category}로 스크롤 완료`);
        }, 150);
      };
      
      // 스크롤 이벤트 리스너 등록
      contentRef.current.addEventListener('scroll', scrollHandler);
      
      // 부드러운 스크롤 실행
      contentRef.current.scrollTo({
        top: offsetPosition,
        behavior: 'smooth'
      });
      
      // 안전장치: 최대 800ms 후에는 무조건 스크롤 플래그 해제
      setTimeout(() => {
        cleanup();
        // console.log(`카테고리 ${category}로 스크롤 타임아웃 완료`);
      }, 800);
    }
  };

  // selectedCategory 변경 감지 및 ref 업데이트
  const lastSelectedCategory = useRef(selectedCategory);

  // 선택된 카테고리가 변경될 때 스크롤 이동
  useEffect(() => {
    // 선택된 카테고리가 변경되었고, 스크롤 중이 아니면 스크롤 이동
    if (selectedCategory !== lastSelectedCategory.current && !isScrolling) {
      lastSelectedCategory.current = selectedCategory;
      scrollToCategory(selectedCategory);
    }
  }, [selectedCategory, isScrolling]);

  // 스크롤 감지 핸들러 - 현재 화면에 보이는 카테고리 판별
  const handleScroll = () => {
    // 스크롤 중인 상태에서는 카테고리 감지를 건너뜀
    if (isScrolling || !contentRef.current) return;
    
    const scrollPosition = contentRef.current.scrollTop;
    const containerHeight = contentRef.current.clientHeight;
    const scrollHeight = contentRef.current.scrollHeight;
    
    // 스크롤이 맨 위에 있는지 확인 (특수 처리)
    if (scrollPosition <= 2) {
      const topCategory = categories[0];
      if (visibleCategory !== topCategory) {
        // console.log("맨 위로 스크롤: 첫 번째 카테고리로 설정", scrollPosition);
        setVisibleCategory(topCategory);
        setSelectedCategory(topCategory);
      }
      return;
    }
    
    // 스크롤이 거의 끝에 도달했는지 확인
    const isNearBottom = scrollPosition + containerHeight >= scrollHeight - 100;
    
    if (isNearBottom) {
      // 스크롤이 거의 끝에 도달했으면 마지막 카테고리로 설정
      const bottomCategory = categories[categories.length - 1];
      if (visibleCategory !== bottomCategory) {
        // console.log("맨 아래로 스크롤: 마지막 카테고리로 설정");
        setVisibleCategory(bottomCategory);
        setSelectedCategory(bottomCategory);
      }
      return;
    }
    
    // 각 카테고리 섹션의 위치를 확인하여 현재 보이는 카테고리 결정
    let currentCategory = null;
    let minDistance = Number.MAX_VALUE;
    
    // 각 카테고리 확인 (가장 가까운 카테고리를 선택)
    categories.forEach(category => {
      const element = categoryRefs.current[category];
      if (!element) return;
      
      const topPosition = element.offsetTop;
      const bottomPosition = topPosition + element.offsetHeight;
      const middlePosition = topPosition + (element.offsetHeight / 2);
      
      // 스크롤 위치와의 거리 계산
      const distance = Math.abs(scrollPosition - topPosition);
      
      // 섹션 내에 있는 경우
      if (scrollPosition >= topPosition - 50 && scrollPosition < bottomPosition) {
        if (!currentCategory || distance < minDistance) {
          currentCategory = category;
          minDistance = distance;
        }
      }
    });
    
    // 적합한 카테고리를 찾지 못했으면 기본값 사용
    if (!currentCategory) {
      // 가장 가까운 카테고리 찾기
      categories.forEach(category => {
        const element = categoryRefs.current[category];
        if (!element) return;
        
        const distance = Math.abs(scrollPosition - element.offsetTop);
        if (distance < minDistance) {
          currentCategory = category;
          minDistance = distance;
        }
      });
    }
    
    // 여전히 없으면 기본값 사용
    if (!currentCategory) {
      currentCategory = categories[0];
    }
    
    if (visibleCategory !== currentCategory) {
      // console.log(`카테고리 변경: ${visibleCategory} -> ${currentCategory} (스크롤 위치: ${scrollPosition})`);
      setVisibleCategory(currentCategory);
      // 스크롤 중이 아닐 때만 사이드바 카테고리 업데이트
      setSelectedCategory(currentCategory);
    }
  };
  
  // 스크롤 이벤트 리스너 등록 - 민감도 향상
  useEffect(() => {
    const scrollContainer = contentRef.current;
    if (scrollContainer) {
      // 디바운스 처리를 위한 변수
      let scrollTimeout = null;
      
      // 스크롤 이벤트 발생 시 실행될 핸들러
      const debouncedHandleScroll = () => {
        // 이미 타이머가 있다면 초기화
        if (scrollTimeout) {
          clearTimeout(scrollTimeout);
        }
        
        // 50ms 후에 스크롤 처리 (부드러운 스크롤 도중에는 너무 많은 이벤트가 발생하지 않도록)
        scrollTimeout = setTimeout(() => {
          handleScroll();
          scrollTimeout = null;
        }, 50);
      };
      
      scrollContainer.addEventListener('scroll', debouncedHandleScroll);
      
      // 컴포넌트 언마운트 시 이벤트 리스너 제거
      return () => {
        scrollContainer.removeEventListener('scroll', debouncedHandleScroll);
        // 타이머 정리
        if (scrollTimeout) {
          clearTimeout(scrollTimeout);
        }
        if (scrollTimerRef.current) {
          clearTimeout(scrollTimerRef.current);
          scrollTimerRef.current = null;
        }
      };
    }
  }, []);

  // WebSocket 데이터가 변경될 때 메뉴 데이터를 갱신
  useEffect(() => {
    // console.log('메뉴 데이터 처리 중:', { 
    //   menuConnected: connected.menu, 
    //   ordersConnected: connected.orders,
    //   menuData: data.menu,
    //   ordersData: data.orders
    // });

    // 로딩 상태 업데이트
    if ((connected.menu && data.menu) || (connected.orders && data.orders?.menuitems)) {
      setIsLoading(false);
    }

    // 메뉴 데이터 추출 및 처리
    const processMenuData = () => {
      try {
        // 메뉴 데이터 소스 결정
        let menuItemsData = [];
        let categoryItems = [];

        // menu 토픽에서 데이터 확인
        if (data.menu && data.menu.items && Array.isArray(data.menu.items)) {
          menuItemsData = data.menu.items;
          
          if (data.menu.categories && Array.isArray(data.menu.categories)) {
            categoryItems = data.menu.categories;
          }
        } 
        // orders 토픽에서 데이터 확인
        else if (data.orders && data.orders.menuitems && Array.isArray(data.orders.menuitems)) {
          menuItemsData = data.orders.menuitems;
          
          if (data.orders.categories && Array.isArray(data.orders.categories)) {
            categoryItems = data.orders.categories;
          }
        }

        if (menuItemsData.length === 0) {
          // console.log('메뉴 데이터를 찾을 수 없음');
          return;
        }

        // 메뉴 아이템 처리
        const processedMenuItems = menuItemsData.map(item => {
          // 위치에 따라 필드 이름이 다를 수 있으므로 확인
          const id = item['MenuItem.id'] || item.id;
          const name = item['MenuItem.name'] || item.name;
          const description = item['MenuItem.description'] || item.description || '';
          const price = item['MenuItem.price'] || item.price;
          const image_url = item['MenuItem.image_url'] || item.image_url || 'default-menu.jpg';
          const categoryId = item['MenuItem.category_id'] || item.category_id || 1;
          const available = item['MenuItem.available'] !== undefined ? item['MenuItem.available'] : 
                          (item.available !== undefined ? item.available : true);
          const prepare_time = item['MenuItem.prepare_time'] || item.prepare_time || 0;
          
          // 카테고리 설정 - 실제 데이터 구조에 맞게 수정
          let category = '추천'; // 기본값
          
          // 메뉴 종류에 따라 카테고리 분류
          if (name.includes('스테이크') || name.includes('파스타') || name.includes('샐러드')) {
            category = '음식';
          } else if (name.includes('주스') || name.includes('와인')) {
            category = '음료';
          }
          // 추천 메뉴 설정
          if (name.includes('샐러드')) {
            category = '추천';
          }

          return {
            id,
            name,
            description,
            price,
            image_url,
            category,
            categoryId,
            available,
            prepare_time,
          };
        });

        setMenuItems(processedMenuItems);
        // console.log('처리된 메뉴 아이템:', processedMenuItems);

        // 카테고리별 메뉴 정리
        const newMenuByCategory = {};
        categories.forEach(cat => {
          newMenuByCategory[cat] = processedMenuItems.filter(item => 
            item.category === cat && item.available
          );
        });
        
        setMenuByCategory(newMenuByCategory);
        // console.log('카테고리별 메뉴:', newMenuByCategory);
      } catch (err) {
        console.error('메뉴 데이터 처리 중 오류 발생:', err);
        setError('메뉴 데이터를 처리하는 중 오류가 발생했습니다.');
      }
    };

    // 메뉴 데이터가 있는 경우 처리
    if (
      (data.menu && (data.menu.items || data.menu.menuitems)) || 
      (data.orders && data.orders.menuitems)
    ) {
      processMenuData();
    }
  }, [data.menu, data.orders, connected.menu, connected.orders, refreshTopic]);

  // 초기 마운트 시 카테고리 초기화
  useEffect(() => {
    // 컴포넌트 마운트 시 기본 카테고리 설정
    setSelectedCategory(categories[0]);
    setVisibleCategory(categories[0]);
    
    // 디바운스 완료 후 카테고리 위치로 스크롤
    const timer = setTimeout(() => {
      if (contentRef.current && categoryRefs.current[categories[0]]) {
        contentRef.current.scrollTop = 0; // 맨 위로 스크롤
      }
    }, 100);
    
    return () => {
      clearTimeout(timer);
    };
  }, []);

  // 알림 추가 함수
  const addNotification = (message) => {
    const id = Date.now();
    setNotifications(prev => [...prev, { id, message }]);
  };

  // 장바구니에 아이템 추가
  const handleAddToCart = (item) => {
    addToCart(item);
    // 성공 알림 추가
    addNotification(`${item.name}이(가) 장바구니에 추가되었습니다.`);
  };

  // 카테고리 선택 처리
  const handleCategorySelect = (category) => {
    // 이미 같은 카테고리가 선택된 경우 재선택 방지
    if (category === selectedCategory) return;
    
    // console.log(`카테고리 선택: ${category}`);
    
    // 스크롤 중일 때는 카테고리 변경을 지연시킴
    if (isScrolling) {
      setTimeout(() => {
        setSelectedCategory(category);
      }, 100);
    } else {
      setSelectedCategory(category);
    }
  };

  // 알림 닫기 처리
  const handleCloseNotification = (id) => {
    setNotifications(notifications.filter(n => n.id !== id));
  };

  // 새로고침 처리
  const handleRefresh = () => {
    setIsLoading(true);
    refreshTopic('menu');
    refreshTopic('orders');
    setNotifications([
      ...notifications, 
      { id: Date.now(), message: '메뉴 정보를 새로고침했습니다.' }
    ]);
  };

  // 로딩 중 표시
  if (isLoading) {
    return (
      <div className="flex h-screen">
        <NotificationOverlay 
          notifications={notifications} 
          onClose={handleCloseNotification} 
        />
        <Sidebar 
          selectedCategory={selectedCategory}
          onSelectCategory={handleCategorySelect}
        />
        <div 
          ref={contentRef} 
          className="flex-grow overflow-auto"
        >
          {/* 로딩 상태 */}
          <div className="flex justify-center items-center h-64">
            <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-[#C49E69]"></div>
            <p className="text-gray-500 text-xl ml-4">메뉴를 불러오는 중입니다...</p>
          </div>
        </div>
        
        <CartSidebar 
          cartItems={cartItems}
          totalAmount={getTotalAmount()}
          onRemove={removeFromCart}
          onIncrease={increaseQuantity}
          onDecrease={decreaseQuantity}
        />
      </div>
    );
  }

  // 오류 표시
  if (error) {
    return (
      <div className="flex h-screen">
        <NotificationOverlay 
          notifications={notifications} 
          onClose={handleCloseNotification} 
        />
        <Sidebar 
          selectedCategory={selectedCategory}
          onSelectCategory={handleCategorySelect}
        />
        <div 
          ref={contentRef} 
          className="flex-grow overflow-auto"
        >
          <div className="bg-red-100 border border-red-400 text-red-700 p-4 rounded-md">
            <p className="text-xl">{error}</p>
            <button
              className="mt-4 bg-[#E53E3E] text-white px-6 py-3 rounded-md text-lg"
              onClick={() => {
                setError(null);
                if (connected.menu) {
                  refreshTopic('menu');
                } else if (connected.orders) {
                  refreshTopic('orders');
                } else {
                  window.location.reload();
                }
              }}
            >
              다시 시도
            </button>
          </div>
        </div>
        
        <CartSidebar 
          cartItems={cartItems}
          totalAmount={getTotalAmount()}
          onRemove={removeFromCart}
          onIncrease={increaseQuantity}
          onDecrease={decreaseQuantity}
        />
      </div>
    );
  }

  // 메뉴 데이터가 없는 경우
  if (Object.values(menuByCategory).flat().length === 0) {
    return (
      <div className="flex h-screen">
        <NotificationOverlay 
          notifications={notifications} 
          onClose={handleCloseNotification} 
        />
        <Sidebar 
          selectedCategory={selectedCategory}
          onSelectCategory={handleCategorySelect}
        />
        <div 
          ref={contentRef} 
          className="flex-grow overflow-auto"
        >
          <div className="flex flex-col items-center justify-center p-6 h-64">
            <div className="bg-yellow-100 text-yellow-800 p-4 rounded-md text-center w-full max-w-md">
              <p className="text-xl mb-4">메뉴 정보 없음</p>
              <p className="mb-4">현재 사용 가능한 메뉴 정보가 없습니다.</p>
              <button
                onClick={handleRefresh}
                className="bg-[#C49E69] text-white px-4 py-2 rounded-md"
              >
                다시 불러오기
              </button>
            </div>
          </div>
        </div>
        
        <CartSidebar 
          cartItems={cartItems}
          totalAmount={getTotalAmount()}
          onRemove={removeFromCart}
          onIncrease={increaseQuantity}
          onDecrease={decreaseQuantity}
        />
      </div>
    );
  }

  return (
    <div className="flex h-screen">
      <NotificationOverlay 
        notifications={notifications} 
        onClose={handleCloseNotification} 
      />
      <Sidebar 
        selectedCategory={selectedCategory}
        onSelectCategory={handleCategorySelect}
      />
      <div 
        ref={contentRef} 
        className="flex-grow overflow-auto"
        onScroll={() => {
          if (scrollTimerRef.current) clearTimeout(scrollTimerRef.current);
          scrollTimerRef.current = setTimeout(() => {
            handleScroll();
            scrollTimerRef.current = null;
          }, 50);
        }}
      >
        {/* 메뉴 섹션 */}
        <div className="pb-20 pt-1">
          {categories.map((cat, index) => (
            <div 
              key={cat} 
              ref={el => categoryRefs.current[cat] = el}
              className={`${index === 0 ? 'pt-4 pb-12' : index === categories.length - 1 ? 'pb-80' : 'pb-20'}`}
              style={{ minHeight: index === 0 ? '50px' : '300px' }}
            >
              <CategoryHeader category={cat} id={cat} />
              <MenuGrid 
                items={menuByCategory[cat] || []} 
                onAddToCart={handleAddToCart} 
                category={cat}
              />
            </div>
          ))}
        </div>
      </div>
      
      {/* 장바구니 사이드바 */}
      <CartSidebar 
        cartItems={cartItems}
        totalAmount={getTotalAmount()}
        onRemove={removeFromCart}
        onIncrease={increaseQuantity}
        onDecrease={decreaseQuantity}
      />
    </div>
  );
};

export default HomePage; 