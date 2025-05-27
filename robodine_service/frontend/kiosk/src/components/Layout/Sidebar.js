import React, { useState } from 'react';
import { useNavigate, useLocation } from 'react-router-dom';
import axios from 'axios';
import { useLanguage } from '../../context/LanguageContext';

const Base_API_URL = process.env.REACT_APP_BASE_URL


// 직원 호출 모달 컴포넌트
const StaffCallModal = ({ isOpen, onClose, onSelect }) => {
  const { t } = useLanguage();
  if (!isOpen) return null;

  const callOptions = [
    { id: 'HELP', label: t('staffCall.options.help'), icon: '🙋‍♂️' },
    { id: 'MENU', label: t('staffCall.options.menu'), icon: '🍽️' },
    { id: 'PAYMENT', label: t('staffCall.options.payment'), icon: '💳' },
    { id: 'BIRTHDAY', label: t('staffCall.options.birthday'), icon: '🎂' },
    { id: 'OTHER', label: t('staffCall.options.other'), icon: '❓' }
  ];

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 z-50 flex items-center justify-center p-4">
      <div className="bg-white rounded-lg shadow-lg w-full max-w-lg p-6">
        <div className="flex justify-between items-center mb-6">
          <h2 className="text-2xl font-bold">{t('staffCall.title')}</h2>
          <button 
            onClick={onClose}
            className="text-gray-500 hover:text-gray-700 text-3xl"
            aria-label={t('staffCall.close')}
          >
            &times;
          </button>
        </div>
        
        <div className="grid grid-cols-2 gap-4">
          {callOptions.map(option => (
            <button
              key={option.id}
              className="flex flex-col items-center justify-center bg-gray-100 hover:bg-gray-200 rounded-lg p-6 transition-colors duration-200"
              onClick={() => onSelect(option)}
            >
              <span className="text-4xl mb-2">{option.icon}</span>
              <span className="text-lg font-medium text-center">{option.label}</span>
            </button>
          ))}
        </div>
      </div>
    </div>
  );
};

// 언어 선택 모달 컴포넌트
const LanguageModal = ({ isOpen, onClose, onSelect }) => {
  const { t } = useLanguage();
  if (!isOpen) return null;

  const languageOptions = [
    { id: 'ko', label: t('language.korean'), icon: '🇰🇷' },
    { id: 'en', label: t('language.english'), icon: '🇺🇸' },
    { id: 'ja', label: t('language.japanese'), icon: '🇯🇵' }
  ];

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 z-50 flex items-center justify-center p-4">
      <div className="bg-white rounded-lg shadow-lg w-full max-w-lg p-6">
        <div className="flex justify-between items-center mb-6">
          <h2 className="text-2xl font-bold">{t('language.title')}</h2>
          <button 
            onClick={onClose}
            className="text-gray-500 hover:text-gray-700 text-3xl"
            aria-label={t('language.close')}
          >
            &times;
          </button>
        </div>
        
        <div className="grid grid-cols-2 gap-4">
          {languageOptions.map(option => (
            <button
              key={option.id}
              className="flex flex-col items-center justify-center bg-gray-100 hover:bg-gray-200 rounded-lg p-6 transition-colors duration-200"
              onClick={() => onSelect(option)}
            >
              <span className="text-4xl mb-2">{option.icon}</span>
              <span className="text-lg font-medium text-center">{option.label}</span>
            </button>
          ))}
        </div>
      </div>
    </div>
  );
};

// 카테고리 버튼 컴포넌트
const CategoryButton = ({ icon, label, selected, active, onClick, className, ...props }) => {
  return (
    <button
      className={`w-40 h-40 flex flex-col items-center justify-center rounded-md transition-colors duration-300 ${
        active
          ? 'bg-[#C49E69] text-white'
          : selected
            ? 'bg-[#E5D5B8] text-[#8A623A]'
            : 'text-gray-700 hover:bg-gray-200'
      } ${className}`}
      onClick={onClick}
      {...props}
    >
      <span className="text-5xl mb-3">{icon}</span>
      <span className="text-2xl font-medium">{label}</span>
    </button>
  );
};

// 기능 버튼 컴포넌트
const FunctionButton = ({ icon, label, color, onClick }) => {
  return (
    <button
      className={`w-full h-20 flex items-center justify-center rounded-md transition-colors duration-200 ${color} text-white my-2`}
      onClick={onClick}
    >
      <span className="text-3xl mr-2">{icon}</span>
      <span className="text-xl font-medium">{label}</span>
    </button>
  );
};

const Sidebar = ({ selectedCategory, onSelectCategory }) => {
  const navigate = useNavigate();
  const location = useLocation();
  const [staffCalled, setStaffCalled] = useState(false);
  const [hasActiveOrder, setHasActiveOrder] = useState(false);
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [isClickDelay, setIsClickDelay] = useState(false); // 빠른 연속 클릭 방지
  const [isLanguageModalOpen, setIsLanguageModalOpen] = useState(false);
  
  // LanguageContext에서 언어 관련 함수들 가져오기
  const { language, changeLanguage, getLanguageIcon, t } = useLanguage();

  // 카테고리 목록 및 아이콘
  const iconMap = {
    '추천': '⭐',
    '음식': '🍽️',
    '음료': '🥤'
  };

  // 번역된 카테고리 목록
  const translatedCategories = [
    { id: '추천', label: t('categories.recommended') },
    { id: '음식', label: t('categories.food') },
    { id: '음료', label: t('categories.beverage') }
  ];

  // 카테고리 클릭 처리
  const handleCategoryClick = (category) => {
    // 이미 클릭 지연 중이면 무시
    if (isClickDelay) return;
    
    // 연속 클릭 방지를 위한 딜레이 설정 (500ms)
    setIsClickDelay(true);
    setTimeout(() => {
      setIsClickDelay(false);
    }, 500);
    
    // 홈 페이지가 아닌 경우 홈으로 이동 후 카테고리 설정
    if (location.pathname !== '/') {
      navigate('/');
      // 홈 페이지 로드 후 카테고리 설정
      setTimeout(() => {
        if (typeof onSelectCategory === 'function') {
          onSelectCategory(category);
        }
      }, 200);
    } else if (typeof onSelectCategory === 'function') {
      // 이미 홈 페이지인 경우 카테고리만 설정
      onSelectCategory(category);
    }
  };

  // 주문 현황 페이지로 이동
  const goOrderStatus = () => navigate('/order-status');
  
  // 직원 호출 모달 열기
  const openStaffCallModal = () => {
    setIsModalOpen(true);
  };

  // 직원 호출 모달 닫기
  const closeStaffCallModal = () => {
    setIsModalOpen(false);
  };
  
  // 직원 호출 유형 선택 처리
  const handleCallTypeSelect = async (option) => {
    // 직원 호출 요청 처리
    const robot_id = null;
    try {
      await axios.post(`${Base_API_URL}/robots/command`, {
        command: option.id,
        parameters: {
          "table_id": 1
        }
      });
    }
    catch (error) {
      console.error('직원 호출 요청 중 오류 발생:', error);
      alert(t('staffCall.error'));
      return;
    }

    setStaffCalled(true);
    closeStaffCallModal();
    alert(t('staffCall.success', { type: option.label }));
    
  // try {
  //   const response = await axios.post(`${API_URL}/orders`, orderData);
  //   return response.data;
  // } catch (error) {
  //   console.error('주문 생성 중 오류 발생:', error);
  //   throw error;
  // }
    // 실제 구현에서는 웹소켓이나 API를 통해 직원 호출 기능 구현
    // 호출 유형(option.id)과 함께 서버로 전송
    
    // 10초 후 호출 상태 초기화 (데모용)
    setTimeout(() => {
      setStaffCalled(false);
    }, 10000);
  };
  
  // 언어 모달 열기
  const openLanguageModal = () => {
    setIsLanguageModalOpen(true);
  };

  // 언어 모달 닫기
  const closeLanguageModal = () => {
    setIsLanguageModalOpen(false);
  };

  // 언어 선택 처리
  const handleLanguageSelect = (option) => {
    // Context를 통해 언어 변경
    changeLanguage(option.id);
    closeLanguageModal();
    alert(t('language.success', { language: option.label }));
  };
  
  const isHomePage = location.pathname === '/';

  return (
    <>
      <aside className="w-48 bg-[#F7F3EE] flex flex-col h-full">
        {/* 카테고리 버튼 영역 */}
        <div className="flex-grow flex flex-col items-center py-8 space-y-8">
          {translatedCategories.map(cat => (
            <CategoryButton
              key={cat.id}
              icon={iconMap[cat.id]}
              label={cat.label}
              selected={selectedCategory === cat.id}
              active={isHomePage && selectedCategory === cat.id}
              onClick={() => handleCategoryClick(cat.id)}
              aria-label={`${cat.label} 메뉴`}
            />
          ))}
        </div>
        
        {/* 기능 버튼 영역 */}
        <div className="p-4 border-t border-gray-300">
          <FunctionButton
            icon="📝"
            label={t('sidebar.orderStatus')}
            color="bg-blue-600 hover:bg-blue-700"
            onClick={goOrderStatus}
          />
          <FunctionButton
            icon="📢"
            label={staffCalled ? t('sidebar.callStaffInProgress') : t('sidebar.callStaff')}
            color="bg-red-600 hover:bg-red-700"
            onClick={openStaffCallModal}
          />
          <FunctionButton
            icon={getLanguageIcon()}
            label={t('sidebar.changeLanguage')}
            color="bg-green-600 hover:bg-green-700"
            onClick={openLanguageModal}
          />
        </div>
      </aside>

      <StaffCallModal 
        isOpen={isModalOpen} 
        onClose={closeStaffCallModal} 
        onSelect={handleCallTypeSelect}
      />

      <LanguageModal
        isOpen={isLanguageModalOpen}
        onClose={closeLanguageModal}
        onSelect={handleLanguageSelect}
      />
    </>
  );
};

export default Sidebar; 