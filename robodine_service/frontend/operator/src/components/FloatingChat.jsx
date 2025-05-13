import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { useWebSockets } from '../contexts/WebSocketContext';
import '../styles/FloatingChat.css';

// 테마 설정 타입
const THEMES = {
  LIGHT: 'light',
  DARK: 'dark',
  SYSTEM: 'system'
};

// 기본 설정값
const DEFAULT_SETTINGS = {
  theme: THEMES.SYSTEM,
  fontSize: 14,
  chatWidth: localStorage.getItem('chatWidth') ? parseInt(localStorage.getItem('chatWidth')) : 350,
  chatHeight: localStorage.getItem('chatHeight') ? parseInt(localStorage.getItem('chatHeight')) : 500,
  primaryColor: '#007bff',
  backgroundColor: '#ffffff',
  messageColor: '#f0f0f0',
  fontFamily: 'system-ui, -apple-system, sans-serif'
};

const FloatingChat = () => {
  // 상태 관리
  const [isOpen, setIsOpen] = useState(() => {
    const saved = localStorage.getItem('chatOpen');
    return saved ? JSON.parse(saved) : false;
  });
  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  const [position, setPosition] = useState(() => {
    const saved = localStorage.getItem('chatPosition');
    return saved ? JSON.parse(saved) : { x: window.innerWidth - 100, y: window.innerHeight - 100 };
  });
  const [settings, setSettings] = useState(() => {
    const saved = localStorage.getItem('chatSettings');
    return saved ? { ...DEFAULT_SETTINGS, ...JSON.parse(saved) } : DEFAULT_SETTINGS;
  });
  const [showSettings, setShowSettings] = useState(false);
  const [optimisticMessages, setOptimisticMessages] = useState([]);
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [page, setPage] = useState(1);
  const [hasMore, setHasMore] = useState(true);
  const [isLoadingMore, setIsLoadingMore] = useState(false);
  const [searchQuery, setSearchQuery] = useState('');
  const [showSearch, setShowSearch] = useState(false);
  const [showImageModal, setShowImageModal] = useState(false);
  const [selectedImage, setSelectedImage] = useState(null);
  const [imageScale, setImageScale] = useState(1);
  const [modalPosition, setModalPosition] = useState({ x: 0, y: 0 });
  const [isModalDragging, setIsModalDragging] = useState(false);
  const [modalDragStart, setModalDragStart] = useState({ x: 0, y: 0 });
  const [imagePosition, setImagePosition] = useState({ x: 0, y: 0 });
  const [isImageDragging, setIsImageDragging] = useState(false);
  const [imageDragStart, setImageDragStart] = useState({ x: 0, y: 0 });
  const [settingsModalPosition, setSettingsModalPosition] = useState({ x: 0, y: 0 });
  const [isSettingsModalDragging, setIsSettingsModalDragging] = useState(false);
  const [settingsModalDragStart, setSettingsModalDragStart] = useState({ x: 0, y: 0 });

  // refs
  const dragRef = useRef(null);
  const resizeRef = useRef(null);
  const dragStartPos = useRef({ x: 0, y: 0 });
  const resizeStartPos = useRef({ width: 0, height: 0 });
  const chatContainerRef = useRef(null);
  const prevMessagesLength = useRef(0);
  const { apiCall } = useAuth();
  const { data } = useWebSockets();

  // 상태 저장
  useEffect(() => {
    localStorage.setItem('chatOpen', JSON.stringify(isOpen));
  }, [isOpen]);

  useEffect(() => {
    localStorage.setItem('chatPosition', JSON.stringify(position));
  }, [position]);

  useEffect(() => {
    localStorage.setItem('chatSettings', JSON.stringify(settings));
  }, [settings]);

  // 테마 적용
  useEffect(() => {
    const root = document.documentElement;
    if (settings.theme === THEMES.SYSTEM) {
      const systemTheme = window.matchMedia('(prefers-color-scheme: dark)').matches ? THEMES.DARK : THEMES.LIGHT;
      root.setAttribute('data-theme', systemTheme);
    } else {
      root.setAttribute('data-theme', settings.theme);
    }
  }, [settings.theme]);

  // 메시지 로드
  const loadMessages = async (pageNum = 1, search = '') => {
    try {
      setIsLoadingMore(true);
      const response = await apiCall(`/api/chat/history?page=${pageNum}&search=${search}`, 'GET');
      console.log('메시지 로드:', response);
      if (response.data) {
        const newMessages = response.data.messages;
        setHasMore(newMessages.length === 20); // 20개씩 로드
        if (pageNum === 1) {
          // 최신 메시지가 아래에 오도록 정렬
          setMessages([...newMessages].reverse());
        } else {
          // 이전 메시지는 위에 추가
          setMessages(prev => [...newMessages.reverse(), ...prev]);
        }
      }
    } catch (error) {
      console.error('메시지 로드 실패:', error);
    } finally {
      setIsLoadingMore(false);
    }
  };

  // 초기 메시지 로드
  useEffect(() => {
    if (isOpen) {
      loadMessages(1, searchQuery);
    }
  }, [isOpen, searchQuery]);

  // 스크롤 이벤트 처리
  const handleScroll = async (e) => {
    if (e.target.scrollTop === 0 && hasMore && !isLoadingMore) {
      setPage(prev => prev + 1);
      await loadMessages(page + 1, searchQuery);
    }
  };

  // 드래그 시작 핸들러
  const handleMouseDown = (e) => {
    if (e.target.className.includes('drag-handle') || 
        e.target.className.includes('chat-button') ||
        e.target.className.includes('chat-icon')) {
      setIsDragging(true);
      dragStartPos.current = {
        x: e.clientX - position.x,
        y: e.clientY - position.y
      };
      e.preventDefault();
    } else if (e.target.className.includes('resize-handle')) {
      setIsResizing(true);
      resizeStartPos.current = {
        width: settings.chatWidth,
        height: settings.chatHeight,
        x: e.clientX,
        y: e.clientY
      };
      e.preventDefault();
    }
  };

  // 리사이즈 핸들러
  const handleResize = (e) => {
    if (isResizing) {
      const deltaX = e.clientX - resizeStartPos.current.x;
      const deltaY = e.clientY - resizeStartPos.current.y;
      const newWidth = Math.max(300, Math.min(resizeStartPos.current.width + deltaX, window.innerWidth - position.x));
      const newHeight = Math.max(400, Math.min(resizeStartPos.current.height + deltaY, window.innerHeight - position.y));
      
      setSettings(prev => ({
        ...prev,
        chatWidth: newWidth,
        chatHeight: newHeight
      }));

      // 크기 변경 시 localStorage에 저장
      localStorage.setItem('chatWidth', newWidth.toString());
      localStorage.setItem('chatHeight', newHeight.toString());
    }
  };

  // 설정 변경 핸들러
  const handleSettingChange = (key, value) => {
    setSettings(prev => ({
      ...prev,
      [key]: value
    }));
  };

  // 검색 핸들러
  const handleSearch = (e) => {
    setSearchQuery(e.target.value);
    setPage(1);
  };

  // 서버에서 받은 메시지와 낙관적 메시지를 병합하여 중복 없이 표시
  useEffect(() => {
    if (!data.chat || !Array.isArray(data.chat)) return;

    // 서버 메시지 시간순 정렬
    const sortedServer = [...data.chat].sort((a, b) => new Date(a.timestamp) - new Date(b.timestamp));
    
    // 낙관적 메시지 중 서버에 없는 것만 필터링
    const pendingMessages = optimisticMessages.filter(opt => 
      !sortedServer.some(msg => 
        msg.question === opt.question && 
        Math.abs(new Date(msg.timestamp) - new Date(opt.timestamp)) < 1000 // 1초 이내의 메시지는 중복으로 간주
      )
    );

    // 최종 메시지 병합 및 정렬
    const mergedMessages = [...sortedServer, ...pendingMessages]
      .sort((a, b) => new Date(a.timestamp) - new Date(b.timestamp));
      // console.log('병합된 메시지:', mergedMessages);

    setMessages(mergedMessages);
  }, [data.chat, optimisticMessages]);

  // 드래그 중 핸들러
  const handleMouseMove = (e) => {
    if (isDragging) {
      const newX = e.clientX - dragStartPos.current.x;
      const newY = e.clientY - dragStartPos.current.y;
      const maxX = window.innerWidth - 80;
      const maxY = window.innerHeight - 80;
      setPosition({
        x: Math.max(0, Math.min(newX, maxX)),
        y: Math.max(0, Math.min(newY, maxY))
      });
    }
  };

  // 드래그 종료 핸들러
  const handleMouseUp = (e) => {
    if (isDragging) {
      setIsDragging(false);
      const deltaX = Math.abs(e.clientX - (position.x + dragStartPos.current.x));
      const deltaY = Math.abs(e.clientY - (position.y + dragStartPos.current.y));
      if (deltaX < 5 && deltaY < 5 && !isOpen && 
         (e.target.className.includes('chat-button') || e.target.className.includes('chat-icon'))) {
        handleChatOpen();
      }
    }
  };

  // 채팅창 열기 핸들러
  const handleChatOpen = () => {
    setIsOpen(true);
  };

  useEffect(() => {
    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging]);

  // 채팅창 토글
  const toggleChat = (e) => {
    e.stopPropagation();
    setIsOpen(!isOpen);
  };

  // 메시지 전송 핸들러
  const handleSendMessage = async () => {
    if (inputMessage.trim() === '') return;
    setIsLoading(true);
    try {
      // 낙관적 메시지 추가 (id를 임시로 부여)
      const now = new Date();
      const optimisticMsg = {
        question: inputMessage,
        timestamp: now.toISOString(),
        pending: true,
        id: `pending-${now.getTime()}`,
        status: 'pending' // 상태 추가
      };
      
      setOptimisticMessages(prev => [...prev, optimisticMsg]);
      setInputMessage('');
      await apiCall('/api/chat/send', 'POST', { message: inputMessage });
      setIsLoading(false);
    } catch (error) {
      setIsLoading(false);
      setOptimisticMessages(prev => [
        ...prev, 
        {
          id: `error-${Date.now()}`,
          answer: '메시지 전송 중 오류가 발생했습니다. 다시 시도해주세요.',
          isError: true,
          timestamp: new Date().toISOString()
        }
      ]);
    }
  };

  // 입력창 엔터키 처리
  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // 채팅 메시지 출력 형식화
  const formatMessage = (message) => {
    if (message.isError) {
      return <div className="error-message">{message.answer}</div>;
    }
 
    // 1) 질문+답변이 모두 있는 경우
    if (message.question && message.answer) {
      return (
        <>
          <div className="message user-message">
            <div className="message-content">
              <HighlightedText text={message.question} searchTerm={searchQuery} />
            </div>
            <div className="message-time">
              {message.timestamp && new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
          <div className="message bot-message">
            <div className="message-content">
              {(() => {
                try {
                  const parsed = JSON.parse(message.answer);
                  return parsed.image ? (
                    <>
                      <div><HighlightedText text={parsed.text} searchTerm={searchQuery} /></div>
                      <img 
                        src={parsed.image} 
                        alt="" 
                        className="chat-image" 
                        onClick={() => handleImageClick(parsed.image)}
                      />
                    </>
                  ) : <HighlightedText text={parsed.text} searchTerm={searchQuery} />;
                } catch {
                  return <HighlightedText text={message.answer} searchTerm={searchQuery} />;
                }
              })()}
            </div>
            <div className="message-time">
              {message.timestamp && new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
        </>
      );
    }
 
    // 2) 사용자 메시지만 있는 경우 (답변 대기 중)
    if (message.question) {
      const isPending = message.pending || 
                       message.status === 'PENDING' || 
                       message.status?.startsWith('RETRYING');
    

      return (
        <>
          <div className={`message user-message ${isPending ? 'pending' : ''}`}>
            <div className="message-content">
              <HighlightedText text={message.question} searchTerm={searchQuery} />
            </div>
            <div className="message-time">
              {message.timestamp && new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
          {isPending && (
            <div className="message bot-message pending">
              <div className="message-content">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
                {message.status?.startsWith('PENDING')
                  ? `답변 하는중...`
                  : `답변 하는중...`}
              </div>
            </div>
          )}
        </>
      );
    }
 
    return null;
  };
 

  // 새 메시지가 추가될 때만 스크롤 내리기
  useEffect(() => {
    if (messages.length > prevMessagesLength.current) {
      if (chatContainerRef.current) {
        chatContainerRef.current.scrollTop = chatContainerRef.current.scrollHeight;
      }
    }
    prevMessagesLength.current = messages.length;
  }, [messages]);

  // 채팅창 열릴 때 채팅창 위치 조정
  useEffect(() => {
    if (isOpen) {
      const minX = 10;
      const minY = 10;
      const maxX = window.innerWidth - settings.chatWidth - 10;
      const maxY = window.innerHeight - settings.chatHeight - 10;
      
      // 채팅창이 화면 밖으로 나가지 않도록 위치 조정
      setPosition(prev => ({
        x: Math.max(minX, Math.min(prev.x, maxX)),
        y: Math.max(minY, Math.min(prev.y, maxY))
      }));
    }
  }, [isOpen, settings.chatWidth, settings.chatHeight]);

  // 검색 결과 하이라이트 컴포넌트
  const HighlightedText = ({ text, searchTerm }) => {
    if (!searchTerm || !text) return <>{text}</>;
    
    const parts = text.split(new RegExp(`(${searchTerm})`, 'gi'));
    return (
      <>
        {parts.map((part, i) => 
          part.toLowerCase() === searchTerm.toLowerCase() ? 
            <mark key={i} className="highlight">{part}</mark> : 
            part
        )}
      </>
    );
  };

  // 검색 결과 이동 함수
  const [currentSearchIndex, setCurrentSearchIndex] = useState(-1);
  const [searchResults, setSearchResults] = useState([]);

  const findSearchResults = () => {
    if (!searchQuery) {
      setSearchResults([]);
      setCurrentSearchIndex(-1);
      return;
    }

    const results = [];
    messages.forEach((msg, index) => {
      if (msg.question?.toLowerCase().includes(searchQuery.toLowerCase())) {
        results.push({ index, type: 'question', text: msg.question });
      }
      if (msg.answer?.toLowerCase().includes(searchQuery.toLowerCase())) {
        results.push({ index, type: 'answer', text: msg.answer });
      }
    });
    
    setSearchResults(results);
    setCurrentSearchIndex(results.length > 0 ? 0 : -1);
  };

  const moveToSearchResult = (direction) => {
    if (searchResults.length === 0) return;
    
    let newIndex = currentSearchIndex + direction;
    if (newIndex >= searchResults.length) newIndex = 0;
    if (newIndex < 0) newIndex = searchResults.length - 1;
    
    setCurrentSearchIndex(newIndex);
    const result = searchResults[newIndex];
    
    // 해당 메시지로 스크롤
    const messageElement = document.querySelector(`[data-message-index="${result.index}"]`);
    if (messageElement) {
      messageElement.scrollIntoView({ behavior: 'smooth', block: 'center' });
      messageElement.classList.add('highlight-message');
      setTimeout(() => messageElement.classList.remove('highlight-message'), 2000);
    }
  };

  // 검색어 변경 시 결과 업데이트
  useEffect(() => {
    findSearchResults();
  }, [searchQuery, messages]);

  // 이미지 모달 관련 함수들
  const handleImageClick = (imageUrl) => {
    setSelectedImage(imageUrl);
    setShowImageModal(true);
    setImageScale(1);
  };

  // 이미지 모달 드래그 핸들러
  const handleModalMouseDown = (e) => {
    if (e.target.className.includes('modal-header')) {
      setIsModalDragging(true);
      setModalDragStart({
        x: e.clientX - modalPosition.x,
        y: e.clientY - modalPosition.y
      });
    }
  };

  const handleModalMouseMove = (e) => {
    if (isModalDragging) {
      const newX = e.clientX - modalDragStart.x;
      const newY = e.clientY - modalDragStart.y;
      setModalPosition({ x: newX, y: newY });
    }
  };

  const handleModalMouseUp = () => {
    setIsModalDragging(false);
  };

  // 이미지 드래그 핸들러
  const handleImageMouseDown = (e) => {
    if (imageScale > 1) {
      setIsImageDragging(true);
      setImageDragStart({
        x: e.clientX - imagePosition.x,
        y: e.clientY - imagePosition.y
      });
    }
  };

  const handleImageMouseMove = (e) => {
    if (isImageDragging && imageScale > 1) {
      const newX = e.clientX - imageDragStart.x;
      const newY = e.clientY - imageDragStart.y;
      setImagePosition({ x: newX, y: newY });
    }
  };

  const handleImageMouseUp = () => {
    setIsImageDragging(false);
  };

  // 이미지 모달 닫을 때 상태 초기화
  const handleImageClose = () => {
    setShowImageModal(false);
    setSelectedImage(null);
    setImageScale(1);
    setImagePosition({ x: 0, y: 0 });
    setModalPosition({ x: 0, y: 0 });
  };

  // 이미지 확대/축소 시 위치 초기화
  const handleImageZoom = (delta) => {
    setImageScale(prev => {
      const newScale = Math.max(0.5, Math.min(3, prev + delta));
      if (newScale <= 1) {
        setImagePosition({ x: 0, y: 0 });
      }
      return newScale;
    });
  };

  const handleImageDownload = async () => {
    if (!selectedImage) return;
    try {
      const response = await fetch(selectedImage);
      const blob = await response.blob();
      const url = window.URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `chat-image-${Date.now()}.png`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      window.URL.revokeObjectURL(url);
    } catch (error) {
      console.error('이미지 다운로드 실패:', error);
    }
  };

  useEffect(() => {
    if (isModalDragging || isImageDragging) {
      document.addEventListener('mousemove', isModalDragging ? handleModalMouseMove : handleImageMouseMove);
      document.addEventListener('mouseup', isModalDragging ? handleModalMouseUp : handleImageMouseUp);
    }
    return () => {
      document.removeEventListener('mousemove', handleModalMouseMove);
      document.removeEventListener('mouseup', handleModalMouseUp);
      document.removeEventListener('mousemove', handleImageMouseMove);
      document.removeEventListener('mouseup', handleImageMouseUp);
    };
  }, [isModalDragging, isImageDragging]);

  // 설정 모달 드래그 핸들러
  const handleSettingsModalMouseDown = (e) => {
    if (e.target.className.includes('modal-header')) {
      setIsSettingsModalDragging(true);
      setSettingsModalDragStart({
        x: e.clientX - settingsModalPosition.x,
        y: e.clientY - settingsModalPosition.y
      });
    }
  };

  const handleSettingsModalMouseMove = (e) => {
    if (isSettingsModalDragging) {
      const newX = e.clientX - settingsModalDragStart.x;
      const newY = e.clientY - settingsModalDragStart.y;
      setSettingsModalPosition({ x: newX, y: newY });
    }
  };

  const handleSettingsModalMouseUp = () => {
    setIsSettingsModalDragging(false);
  };

  // 설정 모달 닫을 때 위치 초기화
  const handleSettingsClose = () => {
    setShowSettings(false);
    setSettingsModalPosition({ x: 0, y: 0 });
  };

  useEffect(() => {
    if (isSettingsModalDragging) {
      document.addEventListener('mousemove', handleSettingsModalMouseMove);
      document.addEventListener('mouseup', handleSettingsModalMouseUp);
    }
    return () => {
      document.removeEventListener('mousemove', handleSettingsModalMouseMove);
      document.removeEventListener('mouseup', handleSettingsModalMouseUp);
    };
  }, [isSettingsModalDragging]);

  // CSS 스타일 추가
  const styles = `
    .message.user-message.pending {
      opacity: 0.6;
    }

    .message.bot-message.pending {
      opacity: 0.8;
      color: var(--primary-color);
    }

    .typing-indicator {
      display: inline-flex;
      align-items: center;
      margin-right: 8px;
    }

    .typing-indicator span {
      width: 4px;
      height: 4px;
      margin: 0 1px;
      background-color: currentColor;
      border-radius: 50%;
      animation: typing 1s infinite ease-in-out;
    }

    .typing-indicator span:nth-child(1) { animation-delay: 0.2s; }
    .typing-indicator span:nth-child(2) { animation-delay: 0.3s; }
    .typing-indicator span:nth-child(3) { animation-delay: 0.4s; }

    @keyframes typing {
      0%, 100% { transform: translateY(0); }
      50% { transform: translateY(-4px); }
    }
  `;

  // 스타일 태그 추가
  useEffect(() => {
    const styleSheet = document.createElement("style");
    styleSheet.innerText = styles;
    document.head.appendChild(styleSheet);
    return () => {
      document.head.removeChild(styleSheet);
    };
  }, []);

  return (
    <>
      <div 
        className={`floating-chat ${isOpen ? 'open' : ''} ${settings.theme}`}
        style={{ 
          left: `${position.x}px`, 
          top: `${position.y}px`,
          width: isOpen ? `${settings.chatWidth}px` : 'auto',
          height: isOpen ? `${settings.chatHeight}px` : 'auto',
          fontSize: `${settings.fontSize}px`,
          fontFamily: settings.fontFamily,
          '--primary-color': settings.primaryColor,
          '--background-color': settings.backgroundColor,
          '--message-color': settings.messageColor
        }}
        onMouseDown={handleMouseDown}
      >
        {isOpen ? (
          <>
            <div className="drag-handle">
              <div className="handle-dots">
                <span></span><span></span><span></span>
              </div>
              <div className="chat-controls">
                <button className="search-button" onClick={() => setShowSearch(!showSearch)}></button>
                <button className="settings-button" onClick={() => setShowSettings(!showSettings)}></button>
                <button className="close-button" onClick={toggleChat}></button>
              </div>
            </div>
            
            {showSearch && (
              <div className="search-bar">
                <input
                  type="text"
                  value={searchQuery}
                  onChange={handleSearch}
                  placeholder="대화 검색..."
                />
                {searchResults.length > 0 && (
                  <div className="search-controls">
                    <span className="search-count">
                      {currentSearchIndex + 1} / {searchResults.length}
                    </span>
                    <button onClick={() => moveToSearchResult(-1)}>▲</button>
                    <button onClick={() => moveToSearchResult(1)}>▼</button>
                  </div>
                )}
              </div>
            )}

            <div 
              className="chat-container" 
              ref={chatContainerRef}
              onScroll={handleScroll}
            >
              {isLoadingMore && (
                <div className="loading-more">이전 대화 불러오는 중...</div>
              )}
              {messages.length === 0 ? (
                <div className="empty-chat">
                  <p>로보다인 챗봇에게 질문하세요!</p>
                </div>
              ) : (
                messages.map((msg, index) => (
                  <div 
                    key={msg.id || `temp-${index}`} 
                    className="message-wrapper"
                    data-message-index={index}
                  >
                    {formatMessage(msg)}
                  </div>
                ))
              )}
            </div>

            <div className="chat-input">
              <textarea
                value={inputMessage}
                onChange={(e) => setInputMessage(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder="메시지를 입력하세요..."
                disabled={isLoading}
              />
              <button 
                onClick={handleSendMessage} 
                disabled={isLoading || inputMessage.trim() === ''}
                className={isLoading ? 'loading' : ''}
              >
                {isLoading ? '전송 중...' : '전송'}
              </button>
            </div>

            <div className="resize-handle" ref={resizeRef}></div>
          </>
        ) : (
          <button className="chat-button">
            <span className="chat-icon">💬</span>
          </button>
        )}
      </div>

      {/* 설정 모달 */}
      {showSettings && (
        <div className="modal-overlay" onClick={handleSettingsClose}>
          <div 
            className={`modal-content settings-modal ${settings.theme}`} 
            onClick={e => e.stopPropagation()}
            style={{ 
              transform: `translate(${settingsModalPosition.x}px, ${settingsModalPosition.y}px)`,
              cursor: isSettingsModalDragging ? 'grabbing' : 'default'
            }}
            onMouseDown={handleSettingsModalMouseDown}
          >
            <div className="modal-header">
              <h3>채팅 설정</h3>
              <button className="modal-close" onClick={handleSettingsClose}>×</button>
            </div>
            <div className="modal-body">
              <div className="setting-item">
                <label>테마</label>
                <select 
                  value={settings.theme} 
                  onChange={(e) => handleSettingChange('theme', e.target.value)}
                  className="theme-select"
                >
                  <option value={THEMES.SYSTEM}>시스템</option>
                  <option value={THEMES.LIGHT}>라이트</option>
                  <option value={THEMES.DARK}>다크</option>
                </select>
              </div>
              <div className="setting-item">
                <label>글자 크기</label>
                <input 
                  type="range" 
                  min="12" 
                  max="20" 
                  value={settings.fontSize}
                  onChange={(e) => handleSettingChange('fontSize', parseInt(e.target.value))}
                />
                <span>{settings.fontSize}px</span>
              </div>
              <div className="setting-item">
                <label>주요 색상</label>
                <input 
                  type="color" 
                  value={settings.primaryColor}
                  onChange={(e) => handleSettingChange('primaryColor', e.target.value)}
                />
              </div>
            </div>
          </div>
        </div>
      )}

      {/* 이미지 모달 */}
      {showImageModal && selectedImage && (
        <div className="modal-overlay" onClick={handleImageClose}>
          <div 
            className={`modal-content image-modal ${settings.theme}`} 
            onClick={e => e.stopPropagation()}
            style={{ 
              transform: `translate(${modalPosition.x}px, ${modalPosition.y}px)`,
              cursor: isModalDragging ? 'grabbing' : 'default'
            }}
            onMouseDown={handleModalMouseDown}
          >
            <div className="modal-header">
              <div className="image-controls">
                <button onClick={() => handleImageZoom(0.1)}>🔍+</button>
                <button onClick={() => handleImageZoom(-0.1)}>🔍-</button>
                <span>{Math.round(imageScale * 100)}%</span>
              </div>
              <div className="image-actions">
                <button onClick={handleImageDownload}>💾 저장</button>
                <button className="modal-close" onClick={handleImageClose}>✕</button>
              </div>
            </div>
            <div className="modal-body image-modal-body">
              <div 
                className="image-container"
                onMouseDown={handleImageMouseDown}
                style={{ cursor: imageScale > 1 ? (isImageDragging ? 'grabbing' : 'grab') : 'default' }}
              >
                <img 
                  src={selectedImage} 
                  alt="확대된 이미지" 
                  style={{ 
                    transform: `scale(${imageScale}) translate(${imagePosition.x}px, ${imagePosition.y}px)`,
                    transition: isImageDragging ? 'none' : 'transform 0.3s ease'
                  }}
                  draggable="false"
                />
              </div>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChat; 