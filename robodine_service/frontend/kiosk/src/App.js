import React, { useState } from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { CartProvider } from './context/CartContext';

// 페이지 컴포넌트들
import HomePage from './pages/HomePage';
import CheckoutPage from './pages/CheckoutPage';
import OrderCompletePage from './pages/OrderCompletePage';
import OrderStatusPage from './pages/OrderStatusPage';
import Layout from './components/Layout/Layout';

function App() {
  const [selectedCategory, setSelectedCategory] = useState('추천');
  const [notifications, setNotifications] = useState([]);

  const handleCloseNotification = (id) => {
    setNotifications(prev => prev.filter(n => n.id !== id));
  };

  return (
    <Router>
      <CartProvider>
        <Routes>
          <Route 
            element={
              <Layout 
                selectedCategory={selectedCategory} 
                onSelectCategory={setSelectedCategory}
                notifications={notifications}
                onCloseNotification={handleCloseNotification}
              />
            }
          >
            <Route 
              path="/" 
              element={
                <HomePage 
                  onSelectCategory={setSelectedCategory} 
                  selectedCategory={selectedCategory}
                  setNotifications={setNotifications}
                />
              } 
            />
            <Route path="/checkout" element={<CheckoutPage />} />
            <Route path="/order-complete" element={<OrderCompletePage />} />
            <Route path="/order-status" element={<OrderStatusPage />} />
          </Route>
        </Routes>
      </CartProvider>
    </Router>
  );
}

export default App; 