// LanguageContext.js
import React, { createContext, useState, useContext, useEffect } from 'react';
import translations from '../locale/translations';

// 언어 컨텍스트 생성
const LanguageContext = createContext();

// 언어 컨텍스트 제공자 컴포넌트
export const LanguageProvider = ({ children }) => {
  // 로컬 스토리지에서 언어 설정 로드 (있으면)
  const [language, setLanguage] = useState(() => {
    const savedLanguage = localStorage.getItem('kioskLanguage');
    return savedLanguage || 'ko'; // 기본값은 한국어
  });

  // 언어 변경시 로컬 스토리지에 저장
  useEffect(() => {
    localStorage.setItem('kioskLanguage', language);
  }, [language]);

  // 언어 변경 함수
  const changeLanguage = (lang) => {
    setLanguage(lang);
  };

  // 현재 언어에 따른 아이콘 가져오기
  const getLanguageIcon = () => {
    switch(language) {
      case 'en': return '🇺🇸';
      case 'ja': return '🇯🇵';
      default: return '🇰🇷';
    }
  };

  // 현재 언어에 따른 라벨 가져오기
  const getLanguageLabel = () => {
    switch(language) {
      case 'en': return '영어';
      case 'ja': return '일본어';
      default: return '한국어';
    }
  };

  // 번역된 텍스트 가져오기 함수
  const t = (key, params = {}) => {
    // 키를 점 표기법에 따라 분리 (예: 'sidebar.orderStatus')
    const keys = key.split('.');
    
    // 언어에 맞는 번역 객체에서 번역 텍스트 찾기
    let translation = translations[language];
    
    // 각 키 단계별로 번역 객체 탐색
    for (const k of keys) {
      if (translation && translation[k]) {
        translation = translation[k];
      } else {
        // 번역이 없으면 키 그대로 반환
        return key;
      }
    }
    
    // 문자열이 아니면 키 그대로 반환
    if (typeof translation !== 'string') {
      return key;
    }
    
    // 매개변수 치환 (예: '{name}님 안녕하세요' -> '홍길동님 안녕하세요')
    let result = translation;
    
    // 모든 매개변수를 번역 문자열에 적용
    Object.keys(params).forEach(paramKey => {
      result = result.replace(`{${paramKey}}`, params[paramKey]);
    });
    
    return result;
  };

  return (
    <LanguageContext.Provider value={{ 
      language, 
      changeLanguage, 
      getLanguageIcon,
      getLanguageLabel,
      t // 번역 함수 추가
    }}>
      {children}
    </LanguageContext.Provider>
  );
};

// 컨텍스트 사용을 위한 커스텀 훅
export const useLanguage = () => {
  const context = useContext(LanguageContext);
  if (!context) {
    throw new Error('useLanguage는 LanguageProvider 내부에서만 사용할 수 있습니다');
  }
  return context;
}; 