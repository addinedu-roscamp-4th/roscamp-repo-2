// HomePage.js
import React, { useState, useEffect, useRef } from 'react';
import MenuGrid from '../components/Menu/MenuGrid';
import { getMenuItems, filterMenuByCategory } from '../api/menuApi';
import { useCart } from '../context/CartContext';
import CartSidebar from '../components/Cart/CartSidebar';

// 카테고리 섹션 헤더 컴포넌트
const CategoryHeader = ({ category, id }) => (
  <div id={`category-${id}`} className="pt-6 pb-4 px-8 border-b-4 border-[#C49E69] bg-[#F7F3EE] mb-4">
    <h2 className="text-3xl font-bold">{category}</h2>
  </div>
);

const HomePage = ({ selectedCategory, onSelectCategory, setNotifications }) => {
  // 상태 관리
  const [menuItems, setMenuItems] = useState([]);
  const [menuByCategory, setMenuByCategory] = useState({});
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [visibleCategory, setVisibleCategory] = useState(selectedCategory || '추천');
  const [isScrolling, setIsScrolling] = useState(false); // 스크롤 중인지 추적
  
  // 카테고리 참조
  const categoryRefs = useRef({});
  // 컨텐츠 영역 참조
  const contentRef = useRef(null);
  // 스크롤 타이머 참조
  const scrollTimerRef = useRef(null);

  // 카테고리 목록
  const categories = ['추천', '음식', '음료'];

  // 카테고리 변경시 스크롤 처리
  const scrollToCategory = (category) => {
    if (categoryRefs.current[category] && contentRef.current) {
      // 스크롤 중임을 표시
      setIsScrolling(true);
      
      // 스크롤 위치 계산
      const headerOffset = 10;
      const elementPosition = categoryRefs.current[category].offsetTop;
      const offsetPosition = elementPosition - headerOffset;
      
      // 부드러운 스크롤 실행
      contentRef.current.scrollTo({
        top: offsetPosition,
        behavior: 'smooth'
      });
      
      // 스크롤 완료 후 상태 업데이트 (약 800ms)
      // 이전 타이머가 있다면 제거
      if (scrollTimerRef.current) {
        clearTimeout(scrollTimerRef.current);
      }
      
      scrollTimerRef.current = setTimeout(() => {
        setIsScrolling(false);
      }, 800);
    }
  };

  // 선택된 카테고리가 변경될 때 스크롤 이동
  useEffect(() => {
    scrollToCategory(selectedCategory);
  }, [selectedCategory]);

  // 스크롤 감지 핸들러 - 현재 화면에 보이는 카테고리 판별
  const handleScroll = () => {
    // 스크롤 중인 상태에서는 카테고리 감지를 건너뜀
    if (isScrolling || !contentRef.current) return;
    
    const scrollPosition = contentRef.current.scrollTop;
    const containerHeight = contentRef.current.clientHeight;
    const scrollHeight = contentRef.current.scrollHeight;
    let currentCategory = categories[0]; // 기본값
    
    // 스크롤이 거의 끝에 도달했는지 확인
    const isNearBottom = scrollPosition + containerHeight >= scrollHeight - 100;
    
    if (isNearBottom) {
      // 스크롤이 거의 끝에 도달했으면 마지막 카테고리로 설정
      currentCategory = categories[categories.length - 1];
    } else {
      // 그렇지 않으면 일반적인 방법으로 현재 카테고리 결정
      for (const category of categories) {
        const element = categoryRefs.current[category];
        if (!element) continue;
        
        const position = element.offsetTop;
        // 현재 스크롤 위치가 해당 카테고리 시작점보다 크거나 같으면 이 카테고리가 현재 보이는 것
        if (scrollPosition >= position - 100) { // 약간의 오프셋(100px) 적용
          currentCategory = category;
        }
      }
    }
    
    if (visibleCategory !== currentCategory) {
      setVisibleCategory(currentCategory);
      // 사이드바 카테고리 버튼 상태 업데이트 (자동 스크롤은 억제)
      if (currentCategory !== selectedCategory) {
        onSelectCategory(currentCategory);
      }
    }
  };
  
  // 스크롤 이벤트 리스너 등록
  useEffect(() => {
    const scrollContainer = contentRef.current;
    if (scrollContainer) {
      scrollContainer.addEventListener('scroll', handleScroll);
      return () => {
        scrollContainer.removeEventListener('scroll', handleScroll);
      };
    }
  }, [visibleCategory, selectedCategory, isScrolling]);

  // 컴포넌트 언마운트 시 타이머 정리
  useEffect(() => {
    return () => {
      if (scrollTimerRef.current) {
        clearTimeout(scrollTimerRef.current);
      }
    };
  }, []);

  // 메뉴 데이터 로드
  useEffect(() => {
    const fetchMenuItems = async () => {
      try {
        setIsLoading(true);
        const data = await getMenuItems();
        setMenuItems(data);
        
        // 카테고리별로 메뉴 분류
        const categorized = {};
        categories.forEach(cat => {
          categorized[cat] = filterMenuByCategory(data, cat);
        });
        setMenuByCategory(categorized);
        
        setIsLoading(false);
      } catch (err) {
        setError('메뉴를 불러오는 중 오류가 발생했습니다.');
        setIsLoading(false);
        // 에러 알림 추가
        addNotification('메뉴를 불러오는 중 오류가 발생했습니다.');
      }
    };

    fetchMenuItems();
  }, []);

  // 장바구니 컨텍스트
  const { addToCart, cartItems, getTotalAmount, removeFromCart, increaseQuantity, decreaseQuantity } = useCart();

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

  return (
    <div className="flex h-full">
      <div 
        ref={contentRef} 
        className="flex-grow overflow-auto"
        onScroll={handleScroll}
      >
        {/* 로딩 상태 */}
        {isLoading && (
          <div className="flex justify-center items-center h-64">
            <p className="text-gray-500 text-xl">메뉴를 불러오는 중입니다...</p>
          </div>
        )}
        
        {/* 에러 상태 */}
        {error && !isLoading && (
          <div className="bg-red-100 border border-red-400 text-red-700 p-4 rounded-md">
            <p className="text-xl">{error}</p>
            <button 
              className="mt-4 bg-[#E53E3E] text-white px-6 py-3 rounded-md text-lg"
              onClick={() => window.location.reload()}
            >
              다시 시도
            </button>
          </div>
        )}
        
        {/* 메뉴 섹션 */}
        {!isLoading && !error && (
          <div className="pb-20">
            {categories.map((cat, index) => (
              <div 
                key={cat} 
                ref={el => categoryRefs.current[cat] = el}
                className={`mb-12 ${index === categories.length - 1 ? 'pb-80' : ''}`} // 마지막 카테고리에 더 큰 하단 패딩 추가
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
        )}
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
};

export default HomePage; 