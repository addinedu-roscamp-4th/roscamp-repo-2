// Header.js
import React, { useState, useEffect } from 'react';

const Header = () => {
  // 현재 시간 상태 관리
  const [currentTime, setCurrentTime] = useState(new Date());
  // 언어 선택 상태 관리 (기본값: 한국어)
  const [language, setLanguage] = useState('ko');

  // 1초마다 시간 업데이트
  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);
    
    // 컴포넌트 언마운트 시 타이머 정리
    return () => clearInterval(timer);
  }, []);

  // 시간을 형식화된 문자열로 변환
  const formattedTime = currentTime.toLocaleTimeString('ko-KR', {
    hour: '2-digit',
    minute: '2-digit',
    hour12: false
  });

  // 언어 변경 핸들러
  const handleLanguageChange = (lang) => {
    setLanguage(lang);
    // 실제 애플리케이션에서는 i18n 라이브러리를 이용해 언어 변경을 처리
  };

  return (
    <header className="bg-indigo-600 text-white p-4 shadow-md">
      <div className="container mx-auto flex justify-between items-center">
        {/* 로고 */}
        <div className="text-xl font-bold">로보다인 키오스크</div>
        
        {/* 현재 시간 */}
        <div className="text-lg font-semibold">{formattedTime}</div>
        
        {/* 언어 선택 */}
        <div className="flex space-x-2">
          <button
            className={`px-2 py-1 rounded ${language === 'ko' ? 'bg-white text-indigo-600' : 'bg-indigo-500'}`}
            onClick={() => handleLanguageChange('ko')}
          >
            한국어
          </button>
          <button
            className={`px-2 py-1 rounded ${language === 'en' ? 'bg-white text-indigo-600' : 'bg-indigo-500'}`}
            onClick={() => handleLanguageChange('en')}
          >
            English
          </button>
        </div>
      </div>
    </header>
  );
};

export default Header; 