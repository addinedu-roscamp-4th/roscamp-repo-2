import React from 'react';
import ReactDOM from 'react-dom/client';
import './index.css';
import App from './App';
import reportWebVitals from './reportWebVitals';

// WebSocket URL 환경 변수 설정
if (!process.env.REACT_APP_WS_URL) {
  // REACT_APP_BASE_URL에서 HTTP URL을 WebSocket URL로 변환
  const baseUrl = process.env.REACT_APP_BASE_URL || 'http://localhost:8000/api';
  // http://domain:port/api -> ws://domain:port/ws
  const wsUrl = baseUrl.replace(/^http/, 'ws').replace(/\/api$/, '/ws');
  window.process = window.process || {};
  window.process.env = window.process.env || {};
  window.process.env.REACT_APP_WS_URL = wsUrl;
}

const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
reportWebVitals(); 