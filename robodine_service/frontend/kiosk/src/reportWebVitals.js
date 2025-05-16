// 웹 활력 지표(Web Vitals) 측정 함수
const reportWebVitals = (onPerfEntry) => {
  if (onPerfEntry && onPerfEntry instanceof Function) {
    import('web-vitals').then(({ getCLS, getFID, getFCP, getLCP, getTTFB }) => {
      getCLS(onPerfEntry); // Cumulative Layout Shift (누적 레이아웃 이동)
      getFID(onPerfEntry); // First Input Delay (첫 입력 지연)
      getFCP(onPerfEntry); // First Contentful Paint (첫 콘텐츠 페인트)
      getLCP(onPerfEntry); // Largest Contentful Paint (최대 콘텐츠 페인트)
      getTTFB(onPerfEntry); // Time to First Byte (첫 바이트까지의 시간)
    });
  }
};

export default reportWebVitals; 