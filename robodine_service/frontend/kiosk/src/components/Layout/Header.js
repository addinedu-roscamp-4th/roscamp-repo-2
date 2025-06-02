// Header.js
import React, { useState, useEffect } from 'react';

// 로고 컴포넌트
const Logo = () => (
  <div className="text-2xl font-bold">로보다인</div>
);

// 시간 표시 컴포넌트
const TimeDisplay = ({ format = "HH:mm" }) => {
  // 현재 시간 상태 관리
  const [currentTime, setCurrentTime] = useState(new Date());

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

  return <div className="text-xl font-medium">{formattedTime}</div>;
};

// 언어 선택기 컴포넌트
const LanguageSelector = () => {
  // 언어 선택 상태 관리 (기본값: 한국어)
  const [language, setLanguage] = useState('ko');

  // 언어 변경 핸들러
  const handleLanguageChange = (lang) => {
    setLanguage(lang);
    // 실제 애플리케이션에서는 i18n 라이브러리를 이용해 언어 변경을 처리
  };

  return (
    <div className="flex space-x-4">
      <button
        className={`px-4 py-2 rounded text-lg ${language === 'ko' ? 'bg-[#C49E69] text-white' : 'bg-white text-gray-700 border border-gray-300'}`}
        onClick={() => handleLanguageChange('ko')}
        aria-label="한국어"
      >
        한국어
      </button>
      <button
        className={`px-4 py-2 rounded text-lg ${language === 'en' ? 'bg-[#C49E69] text-white' : 'bg-white text-gray-700 border border-gray-300'}`}
        onClick={() => handleLanguageChange('en')}
        aria-label="영어"
      >
        English
      </button>
    </div>
  );
};

const Header = () => {
  return (
    <footer className="flex items-center justify-between px-6 py-4 bg-white shadow-inner border-t border-gray-200 h-20">
      <Logo />
      <TimeDisplay format="HH:mm" />
      <LanguageSelector />
    </footer>
  );
};

export default Header; 