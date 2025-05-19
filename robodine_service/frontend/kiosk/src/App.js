import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { CartProvider } from './context/CartContext';
import { UnifiedWebSocketProvider } from './context/UnifiedWebSocketProvider';
import CleaningButtonComponent from './components/CleaningButton';

// 페이지 컴포넌트들
import HomePage from './pages/HomePage';
import CheckoutPage from './pages/CheckoutPage';
import OrderCompletePage from './pages/OrderCompletePage';
import OrderStatusPage from './pages/OrderStatusPage';

function App() {
  // 키오스크에 필요한、사용할 WebSocket 토픽 지정
  const topics = ['menu', 'orders', 'tables'];

  return (
    <UnifiedWebSocketProvider topics={topics}>
      <CartProvider>
        <Router>
          {/* 청소 완료 버튼 - 모든 페이지에서 표시 */}
          <CleaningButtonComponent />
          <Routes>
            <Route index element={<HomePage />} />
            <Route path="/order-status" element={<OrderStatusPage />} />
            <Route path="/checkout" element={<CheckoutPage />} />
            <Route path="/order-complete" element={<OrderCompletePage />} />
          </Routes>
        </Router>
      </CartProvider>
    </UnifiedWebSocketProvider>
  );
}

export default App; 