import React, { useState, useEffect, useCallback } from 'react';
import { 
  Database, AlertTriangle, Search, PlusCircle, 
  Edit, Trash2, Download, Filter, RefreshCw,
  X, Save, CheckSquare, Coffee, Package
} from 'lucide-react';
import { useNavigate } from 'react-router-dom';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useWebSockets } from '../contexts/WebSocketContext';

// 웹소켓 연결 설정
const WS_BASE_URL = process.env.REACT_APP_WS_URL || 'ws://192.168.0.156:8000/ws';

// 웹소켓 연결 함수
const useWebSocket = (topic, onMessageReceived) => {
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  
  useEffect(() => {
    const ws = new WebSocket(`${WS_BASE_URL}/${topic}`);
    
    ws.onopen = () => {
      console.log(`${topic} 웹소켓 연결됨`);
      setConnected(true);
      setError(null);
    };
    
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.type === 'update' && data.topic === topic) {
          onMessageReceived(data.data);
        }
      } catch (error) {
        console.error(`${topic} 웹소켓 메시지 처리 오류:`, error);
      }
    };
    
    ws.onclose = () => {
      console.log(`${topic} 웹소켓 연결 종료`);
      setConnected(false);
    };
    
    ws.onerror = (error) => {
      console.error(`${topic} 웹소켓 오류:`, error);
      setError(`연결 오류: ${error.message || '알 수 없는 오류'}`);
    };
    
    return () => {
      ws.close();
    };
  }, [topic, onMessageReceived]);
  
  return { connected, error };
};

const InventoryPage = () => {
  const [inventory, setInventory] = useState([]);
  const [menuItems, setMenuItems] = useState([]);
  const [menuIngredients, setMenuIngredients] = useState([]);
  const [filteredInventory, setFilteredInventory] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [searchTerm, setSearchTerm] = useState('');
  const [sortConfig, setSortConfig] = useState({ key: 'name', direction: 'ascending' });
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [currentItem, setCurrentItem] = useState(null);
  const [isDeleteModalOpen, setIsDeleteModalOpen] = useState(false);
  const [itemToDelete, setItemToDelete] = useState(null);
  const [adminSettings, setAdminSettings] = useState({});
  const { apiCall } = useAuth();
  const { data, connected } = useWebSockets();
  const navigate = useNavigate();
  const [isIngredientModalOpen, setIsIngredientModalOpen] = useState(false);
  const [allIngredients, setAllIngredients] = useState([]);
  const [selectedExistingIngredient, setSelectedExistingIngredient] = useState('');
  const [isIngredientListModal, setIsIngredientListModal] = useState(false);
  const [filteredIngredients, setFilteredIngredients] = useState([]);
  const [ingredientSearchTerm, setIngredientSearchTerm] = useState('');
  const [deleteType, setDeleteType] = useState('');

  const statusOptions = ['IN_STOCK', 'LOW_STOCK', 'OUT_OF_STOCK'];

  // 초기 데이터 및 설정 로딩
  const fetchInitialData = useCallback(async () => {
    setIsLoading(true);
    setError(null);
    
    try {
      // 메뉴 항목 데이터 가져오기
      const menuItemsData = await apiCall('/api/menu/items');
      setMenuItems(menuItemsData);
      
      // 메뉴 재료 데이터 가져오기
      const menuIngredientsData = await apiCall('/api/menu/ingredients');
      const normalized = menuIngredientsData.map(ing => ({
        id: ing["MenuIngredient.id"] ?? ing.id,            
        name: ing["MenuIngredient.name"] ?? ing.name,
        menu_item_id: ing["MenuIngredient.menu_item_id"] ?? ing.menu_item_id,
        quantity_required: ing["MenuIngredient.quantity_required"] ?? ing.quantity_required
      }));
      console.log('Normalized Menu Ingredients:', normalized);
      setMenuIngredients(normalized);   
      
      // 모든 재료 목록 생성 (중복 제거)
      const uniqueIngredients = Array.from(new Set(
        menuIngredientsData.map(ing => ing.name)
      )).sort();
      setAllIngredients(uniqueIngredients);

      // 관리자 설정 가져오기 (재고 임계값)
      const settings = await apiCall('/api/settings');
      setAdminSettings(settings);
      
      setIsLoading(false);
    } catch (err) {
      console.error('초기 데이터 로드 실패:', err);
      setError('데이터를 불러올 수 없습니다');
      setIsLoading(false);
    }
  }, [apiCall]);

  // 초기 데이터 로딩
  useEffect(() => {
    fetchInitialData();
  }, [fetchInitialData]);

  // 웹소켓 데이터 처리
  useEffect(() => {
    if (data.inventory) {
      setIsLoading(false);
      
      // 데이터 형식에 따라 처리
      let inventoryItems = [];
      if (Array.isArray(data.inventory)) {
        inventoryItems = data.inventory;
      } else if (data.inventory.inventory && Array.isArray(data.inventory.inventory)) {
        inventoryItems = data.inventory.inventory;
        
        // Admin 설정이 있으면 저장
        if (data.inventory.admin_settings) {
          setAdminSettings(data.inventory.admin_settings);
        }
      }
      
      // 인벤토리 데이터 처리를 위한 임시 변수
      const processedInventory = inventoryItems.map(item => {
        // 연관된 메뉴 재료 찾기
        const ingredient = menuIngredients.find(ing => ing.id === item.ingredient_id || ing.name === item.name);
        
        // 연관된 메뉴 항목 찾기
        let menuItem = null;
        if (ingredient && ingredient.menu_item_id) {
          menuItem = menuItems.find(menu => menu.id === ingredient.menu_item_id);
        }
        
        return {
          ...item,
          ingredient,
          menuItem,
          // 재고 임계값은 max_count 또는 관리자 설정에서 가져옴
          threshold: adminSettings.inventory_threshold,
          price: menuItem ? menuItem.price : 0
        };
      });
      
      setInventory(processedInventory);
    }
  }, [data.inventory, menuIngredients, menuItems, adminSettings]);

  // 메뉴 데이터 처리를 위한 별도의 useEffect
  useEffect(() => {
    if (data.menu) {
      // 메뉴 재료 데이터 가져오기
      if (Array.isArray(data.menu.ingredients)) {
        const ingredients = data.menu.ingredients.map(ingredient => {
          return {
            id: ingredient["MenuIngredient.id"] || ingredient.id,
            name: ingredient["MenuIngredient.name"] || ingredient.name,
            menu_item_id: ingredient["MenuIngredient.menu_item_id"] || ingredient.menu_item_id,
            quantity_required: ingredient["MenuIngredient.quantity_required"] || ingredient.quantity_required
          };
        });
        
        setMenuIngredients(ingredients);
        
        // 모든 재료 목록 업데이트 (중복 제거)
        const uniqueIngredients = Array.from(new Set(
          ingredients.map(ing => ing.name)
        )).sort();
        setAllIngredients(uniqueIngredients);
      }
      
      // 메뉴 항목 데이터 가져오기
      if (Array.isArray(data.menu.items)) {
        const items = data.menu.items.map(item => {
          return {
            id: item["MenuItem.id"] || item.id,
            name: item["MenuItem.name"] || item.name,
            price: item["MenuItem.price"] || item.price,
            prepare_time: item["MenuItem.prepare_time"] || item.prepare_time
          };
        });
        
        setMenuItems(items);
      }
    }
  }, [data.menu]);

  // 필터링 및 정렬 적용
  useEffect(() => {
    // 검색어 필터 적용
    let result = [...inventory];
    
    if (searchTerm) {
      result = result.filter(item => 
        (item.name && item.name.toLowerCase().includes(searchTerm.toLowerCase())) ||
        (item.ingredient && item.ingredient.name && item.ingredient.name.toLowerCase().includes(searchTerm.toLowerCase())) ||
        (item.menuItem && item.menuItem.name && item.menuItem.name.toLowerCase().includes(searchTerm.toLowerCase()))
      );
    }
    
    // 정렬 적용
    if (sortConfig.key) {
      result.sort((a, b) => {
        if (a[sortConfig.key] < b[sortConfig.key]) {
          return sortConfig.direction === 'ascending' ? -1 : 1;
        }
        if (a[sortConfig.key] > b[sortConfig.key]) {
          return sortConfig.direction === 'ascending' ? 1 : -1;
        }
        return 0;
      });
    }
    
    setFilteredInventory(result);
  }, [inventory, searchTerm, sortConfig]);

  // 정렬 처리
  const handleSort = (key) => {
    let direction = 'ascending';
    if (sortConfig.key === key && sortConfig.direction === 'ascending') {
      direction = 'descending';
    }
    setSortConfig({ key, direction });
  };

  // 재고 상태에 따른 스타일 클래스
  const getStockStatusClass = (status) => {
    if (status === 'OUT_OF_STOCK') return 'bg-red-100 text-red-800';
    if (status === 'LOW_STOCK') return 'bg-yellow-100 text-yellow-800';
    return 'bg-green-100 text-green-800';
  };

  // 재료 관리 모달 열기
  const handleAddIngredient = () => {
    clearError();
    setIsIngredientListModal(true);
    setIngredientSearchTerm('');
  };

  // 재료 목록 필터링
  useEffect(() => {
    if (menuIngredients.length > 0) {
      let result = [...menuIngredients];
      
      if (ingredientSearchTerm) {
        result = result.filter(ingredient => 
          ingredient.name.toLowerCase().includes(ingredientSearchTerm.toLowerCase())
        );
      }
      
      // 중복 재료 이름 제거 (id 기준으로 가장 최신 항목만 표시)
      const uniqueIngredients = [];
      const uniqueNames = new Set();
      
      // 역순으로 순회하여 가장 최근 항목을 먼저 확인
      [...result].reverse().forEach(ingredient => {
        if (!uniqueNames.has(ingredient.name)) {
          uniqueNames.add(ingredient.name);
          uniqueIngredients.unshift(ingredient); // 원래 순서대로 다시 추가
        }
      });
      
      setFilteredIngredients(uniqueIngredients);
    }
  }, [menuIngredients, ingredientSearchTerm]);

  // 재료 추가 모달 열기
  const handleAddNewIngredient = () => {
    clearError();
    setIsIngredientListModal(false);
    setCurrentItem({
      name: '',
      menu_item_id: '',
      quantity_required: 1
    });
    setSelectedExistingIngredient('');
    setIsIngredientModalOpen(true);
  };

  // 오류 상태 초기화
  const clearError = () => {
    setError(null);
  };

  // 재료 목록 모달 닫기
  const closeIngredientListModal = () => {
    setIsIngredientListModal(false);
    clearError();
  };

  // 재료 추가/수정 모달 닫기
  const closeIngredientModal = () => {
    setIsIngredientModalOpen(false);
    clearError();
  };

  // 삭제 확인 모달 닫기
  const closeDeleteModal = () => {
    setIsDeleteModalOpen(false);
    clearError();
  };

  // 재료 수정 모달 열기
  const handleEditIngredient = (ingredient) => {
    clearError();
    
    if (!ingredient || !ingredient.id) {
      setError('수정할 재료의 ID가 없습니다');
      return;
    }
    
    setIsIngredientListModal(false);
    setCurrentItem({
      id: ingredient.id,
      name: ingredient.name,
      menu_item_id: ingredient.menu_item_id,
      quantity_required: ingredient.quantity_required
    });
    setSelectedExistingIngredient('');
    setIsIngredientModalOpen(true);
  };

  // 재료 저장 (추가/수정)
  const handleSaveIngredient = async () => {
    try {
      clearError();
      
      // 필수 필드 검증
      if (!currentItem.name || !currentItem.name.trim()) {
        setError('재료 이름을 입력해 주세요');
        return;
      }
      
      if (!currentItem.menu_item_id) {
        setError('연결할 메뉴를 선택해 주세요');
        return;
      }
      
      const trimmedName = currentItem.name.trim();
      
      // 중복 재료 검사 (수정 시에도 다른 이름으로 바꿀 경우 체크)
      const isDuplicate = menuIngredients.some(
        ing => ing.name && 
        ing.name.toLowerCase() === trimmedName.toLowerCase() && 
        (!currentItem.id || ing.id !== currentItem.id)
      );
      
      if (isDuplicate) {
        setError(`'${trimmedName}' 재료가 이미 존재합니다. 다른 이름을 사용하세요.`);
        return;
      }
      
      // 메뉴 아이템 ID가 숫자인지 확인
      const menuItemId = parseInt(currentItem.menu_item_id);
      if (isNaN(menuItemId) || menuItemId <= 0) {
        setError('유효한 메뉴를 선택해 주세요');
        return;
      }
      
      // 필요 수량이 숫자인지 확인
      const quantityRequired = parseInt(currentItem.quantity_required || 1);
      if (isNaN(quantityRequired) || quantityRequired <= 0) {
        setError('유효한 필요 수량을 입력해 주세요');
        return;
      }
      
      const payload = {
        name: trimmedName,
        menu_item_id: menuItemId,
        quantity_required: quantityRequired
      };
      
      console.log('재료 저장 페이로드:', payload);
      
      try {
        let response;
        let newItem;
        
        if (currentItem.id) {
          // 기존 재료 수정
          response = await apiCall(`/api/menu/ingredients/${currentItem.id}`, 'PUT', payload);
          
          // 로컬 상태 업데이트
          newItem = { ...payload, id: currentItem.id };
          setMenuIngredients(prev => 
            prev.map(item => item.id === currentItem.id ? newItem : item)
          );
        } else {
          // 새 재료 추가
          try {
            response = await apiCall('/api/menu/ingredients', 'POST', payload);
            
            // 응답에서 새 항목 ID 가져오기
            if (response && response.id) {
              newItem = { ...payload, id: response.id };
            } else {
              // ID가 없는 경우 임시 ID 생성
              newItem = { ...payload, id: Date.now() };
            }
            
            // 새 항목 로컬 상태에 추가
            setMenuIngredients(prev => [...prev, newItem]);
          } catch (postError) {
            console.error('POST 요청 실패:', postError);
            throw postError; // PUT 시도를 제거하고 바로 오류 처리로 넘김
          }
        }
        
        console.log('재료 저장 응답:', response);
        
        // 성공 시 데이터 다시 로드 시도
        try {
          await fetchInitialData();
        } catch (fetchError) {
          console.error('데이터 다시 로드 실패:', fetchError);
          // 실패해도 계속 진행 (이미 로컬 상태는 업데이트됨)
        }
        
        // 모달 닫기
        closeIngredientModal();
        setCurrentItem(null);
        
        // 성공 시 재료 목록 모달로 돌아가기
        closeIngredientListModal();
        
      } catch (apiError) {
        // API 응답 오류 처리
        console.error('재료 저장 실패:', apiError);
        
        if (apiError.message && apiError.message.includes('422')) {
          setError('같은 이름의 재료가 이미 존재합니다. 다른 이름을 사용하세요.');
        } else if (apiError.message && apiError.message.includes('405')) {
          // Method Not Allowed 오류 처리
          setError('API 메서드 오류: 서버에서 이 작업을 지원하지 않습니다.');
        } else if (apiError.message && apiError.message.includes('CORS')) {
          setError('CORS 오류: 서버 연결에 문제가 있습니다.');
        } else {
          setError('재료 저장 중 오류가 발생했습니다. 다시 시도해주세요.');
        }
      }
    } catch (error) {
      console.error('재료 저장 실패:', error);
      setError('재료 저장 중 오류가 발생했습니다');
    }
  };

  // 재고 항목 수정 모달 열기
  const handleEditItem = (item) => {
    setCurrentItem({
      id: item.id,
      ingredient_id: item.ingredient_id,
      name: item.name,
      count: item.count,
      max_count: item.max_count,
      status: item.status
    });
    setIsModalOpen(true);
  };

  // 재료 항목 삭제 모달 열기
  const handleDeleteConfirm = (item, type = 'inventory') => {
    clearError();
    
    if (!item || item.id == null) {
      setError('삭제할 항목의 ID가 없습니다');
      return;
    }
    
    setItemToDelete(item);
    setDeleteType(type || 'inventory');
    setIsDeleteModalOpen(true);
  };

  // 재료 항목 삭제 실행
  const handleDeleteItem = async () => {
    if (!itemToDelete || itemToDelete.id == null) {
      setError('삭제할 항목의 ID가 없습니다');
      closeDeleteModal();
      return;
    }
    
    try {
      if (deleteType === 'ingredient') {
        console.log('재료 삭제:', itemToDelete);
        
        try {
          // 먼저 DELETE 요청을 시도
          await apiCall(`/api/menu/ingredients/${itemToDelete.id}`, 'DELETE');
        } catch (deleteError) {
          console.error('DELETE 요청 실패:', deleteError);
          
          // DELETE가 실패하면(405 Method Not Allowed 등) PUT으로 시도
          if (deleteError.message && deleteError.message.includes('405')) {
            console.log('DELETE 메서드 허용되지 않음, PUT으로 시도');
            // PUT 요청으로 deleted=true 설정
            await apiCall(`/api/menu/ingredients/${itemToDelete.id}`, 'PUT', {
              deleted: true,
              name: itemToDelete.name,
              menu_item_id: itemToDelete.menu_item_id,
              quantity_required: itemToDelete.quantity_required || 1
            });
          } else {
            // 다른 오류라면 그대로 throw
            throw deleteError;
          }
        }
        
        // 성공 시 로컬에서도 항목 삭제
        setMenuIngredients(prev => prev.filter(item => item.id !== itemToDelete.id));
        setFilteredIngredients(prev => prev.filter(item => item.id !== itemToDelete.id));
      } else {
        // 재고 항목 삭제
        await apiCall(`/api/inventory/${itemToDelete.id}`, 'DELETE');
      }
      
      closeDeleteModal();
      setItemToDelete(null);
      
      // 재료를 삭제한 경우 재료 목록 모달로 돌아가기
      if (deleteType === 'ingredient') {
        closeIngredientListModal();
      }
      
      // 성공적으로 삭제 후 데이터 다시 로드
      fetchInitialData();
      
    } catch (error) {
      console.error(`아이템 삭제 실패 (ID: ${itemToDelete.id}):`, error);
      
      if (deleteType === 'ingredient') {
        // 오류 메시지 표시
        if (error.message && error.message.includes('405')) {
          setError('API 메서드 오류: 서버에서 이 삭제 방식을 지원하지 않습니다');
        } else if (error.message && error.message.includes('CORS')) {
          setError('CORS 오류: 서버 연결에 문제가 있습니다');
        } else {
          setError('재료 삭제 중 오류가 발생했습니다');
        }
        
        // CORS 또는 서버 오류가 발생한 경우에도 UI에서만이라도 삭제 처리
        setMenuIngredients(prev => prev.filter(item => item.id !== itemToDelete.id));
        setFilteredIngredients(prev => prev.filter(item => item.id !== itemToDelete.id));
        
        // 삭제 모달 닫기
        closeDeleteModal();
        setItemToDelete(null);
        
        // 재료 목록 모달로 돌아가기
        closeIngredientListModal();
      } else {
        setError('아이템 삭제 중 오류가 발생했습니다');
      }
    }
  };

  // 재료 삭제 처리를 위한 도우미 함수
  const removeIngredientLocally = (id) => {
    if (!id) return;
    setMenuIngredients(prev => prev.filter(item => item.id !== id));
    setFilteredIngredients(prev => prev.filter(item => item.id !== id));
  };

  // 재고 항목 저장 (수정)
  const handleSaveItem = async () => {
    try {
      clearError();
      
      // 필수 필드 검증
      if (!currentItem.count && currentItem.count !== 0) {
        setError('재고량을 입력해 주세요');
        return;
      }
      
      if (!currentItem.max_count) {
        setError('최대 재고량을 입력해 주세요');
        return;
      }
      
      const payload = {
        ingredient_id: currentItem.ingredient_id,
        name: currentItem.name,
        count: parseInt(currentItem.count),
        max_count: parseInt(currentItem.max_count),
        // status는 백엔드에서 자동으로 계산되도록 함
        status: currentItem.status
      };
      
      if (currentItem.id) {
        // 기존 항목 수정
        await apiCall(`/api/inventory/${currentItem.id}`, 'PUT', payload);
        
        // 웹소켓으로 데이터가 업데이트될 때까지 UI 미리 업데이트
        setInventory(prev => prev.map(item => 
          item.id === currentItem.id ? { ...item, ...payload } : item
        ));
      } 
      
      // 모달 닫기
      setIsModalOpen(false);
      setCurrentItem(null);
      // 저장 후 웹소켓으로 자동 갱신됨
    } catch (error) {
      console.error('아이템 저장 실패:', error);
      setError('아이템 저장 중 오류가 발생했습니다');
    }
  };

  // CSV 내보내기
  const exportToCSV = () => {
    // CSV 헤더 생성
    const headers = ['ID', '재료명', '이름', '연결된 메뉴', '재고량', '최대재고량', '상태'];
    
    // 데이터 행 생성
    const rows = filteredInventory.map(item => [
      item.id,
      item.ingredient?.name || '-',
      item.name,
      item.menuItem?.name || '-',
      item.count,
      item.max_count || item.threshold,
      item.status
    ]);
    
    // CSV 문자열 생성
    const csvContent = [
      headers.join(','),
      ...rows.map(row => row.join(','))
    ].join('\n');
    
    // 다운로드 링크 생성
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.setAttribute('href', url);
    link.setAttribute('download', `재고_목록_${new Date().toISOString().slice(0, 10)}.csv`);
    document.body.appendChild(link);
    
    // 다운로드 실행
    link.click();
    
    // 정리
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  return (
    <Layout>
      <div className="container mx-auto p-4">
        <div className="flex justify-between items-center mb-6">
          <h1 className="text-2xl font-bold text-gray-800 flex items-center">
            <Database className="text-blue-600 mr-2" size={28} />
            재고 관리
          </h1>
          
          <div className="flex space-x-2">
            <button 
              onClick={exportToCSV}
              className="flex items-center px-3 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
            >
              <Download size={16} className="mr-2" />
              CSV 내보내기
            </button>
            <button 
              onClick={handleAddIngredient}
              className="flex items-center px-3 py-2 bg-blue-600 rounded-md shadow-sm text-sm font-medium text-white hover:bg-blue-700"
            >
              <Package size={16} className="mr-2" />
              재료 관리
            </button>
          </div>
        </div>

        {error && (
          <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6 flex items-center">
            <AlertTriangle className="mr-2" size={20} />
            <span>{error}</span>
          </div>
        )}
        
        <div className="bg-white rounded-lg shadow overflow-hidden">
          <div className="p-4 border-b flex flex-col md:flex-row justify-between space-y-2 md:space-y-0">
            {/* 검색 */}
            <div className="relative flex-1 max-w-md">
              <input
                type="text"
                placeholder="재고 검색..."
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                className="w-full pl-10 pr-4 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
              />
              <div className="absolute inset-y-0 left-0 flex items-center pl-3">
                <Search size={18} className="text-gray-400" />
              </div>
            </div>
            
            {/* 필터 */}
            <div className="flex space-x-2">
              <button 
                onClick={fetchInitialData}
                className="p-2 border border-gray-300 rounded-md shadow-sm text-gray-700 bg-white hover:bg-gray-50"
                title="새로고침"
              >
                <RefreshCw size={18} />
              </button>
            </div>
          </div>
          
          {isLoading ? (
            <div className="flex items-center justify-center h-64">
              <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
            </div>
          ) : filteredInventory.length === 0 ? (
            <div className="text-center py-12">
              <Database className="mx-auto h-12 w-12 text-gray-400" />
              <h3 className="mt-2 text-sm font-medium text-gray-900">재고 없음</h3>
              <p className="mt-1 text-sm text-gray-500">
                {searchTerm 
                  ? '검색 조건에 맞는 재고 항목이 없습니다.' 
                  : '등록된 재고 항목이 없습니다.'}
              </p>
              <div className="mt-6">
                <button
                  onClick={handleAddIngredient}
                  className="inline-flex items-center px-4 py-2 border border-transparent shadow-sm text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700"
                >
                  <Package className="-ml-1 mr-2 h-5 w-5" aria-hidden="true" />
                  메뉴 재료 관리하기
                </button>
              </div>
            </div>
          ) : (
            <div className="overflow-x-auto">
              <table className="min-w-full divide-y divide-gray-200">
                <thead className="bg-gray-50">
                  <tr>
                    <th 
                      onClick={() => handleSort('name')}
                      className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer hover:bg-gray-100"
                    >
                      이름
                      {sortConfig.key === 'name' && (
                        <span className="ml-1">
                          {sortConfig.direction === 'ascending' ? '↑' : '↓'}
                        </span>
                      )}
                    </th>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      연결된 메뉴
                    </th>
                    <th
                      onClick={() => handleSort('count')}
                      className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer hover:bg-gray-100"
                    >
                      재고량
                      {sortConfig.key === 'count' && (
                        <span className="ml-1">
                          {sortConfig.direction === 'ascending' ? '↑' : '↓'}
                        </span>
                      )}
                    </th>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      상태
                    </th>
                    <th className="px-6 py-3 text-right text-xs font-medium text-gray-500 uppercase tracking-wider">
                      작업
                    </th>
                  </tr>
                </thead>
                <tbody className="bg-white divide-y divide-gray-200">
                  {filteredInventory.map((item) => (
                    <tr key={item.id} className="hover:bg-gray-50">
                      <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">
                        {item.name}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {item.menuItem?.name || '-'}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap">
                        <span className={`px-2 inline-flex text-xs leading-5 font-semibold rounded-full ${getStockStatusClass(item.status)}`}>
                          {item.count} / {item.max_count}
                        </span>
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {item.status === 'IN_STOCK' && '정상'}
                        {item.status === 'LOW_STOCK' && '부족'}
                        {item.status === 'OUT_OF_STOCK' && '없음'}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-right text-sm font-medium">
                        <div className="flex space-x-2 justify-end">
                          <button
                            onClick={() => handleEditItem(item)}
                            className="text-indigo-600 hover:text-indigo-900 focus:outline-none"
                          >
                            <Edit size={18} />
                          </button>
                          <button
                            onClick={() => handleDeleteConfirm(item)}
                            className="text-red-600 hover:text-red-900 focus:outline-none"
                          >
                            <Trash2 size={18} />
                          </button>
                        </div>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
        </div>
      </div>

      {/* 재고 항목 수정 모달 */}
      {isModalOpen && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
            <div className="flex justify-between items-center border-b px-6 py-4">
              <h3 className="text-lg font-medium text-gray-900">
                재고 항목 수정
              </h3>
              <button
                onClick={() => setIsModalOpen(false)}
                className="text-gray-400 hover:text-gray-500"
              >
                <X size={20} />
              </button>
            </div>
            
            <div className="px-6 py-4">
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  재고 이름
                </label>
                <input
                  type="text"
                  readOnly
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm bg-gray-100"
                  value={currentItem.name}
                />
                <p className="mt-1 text-xs text-gray-500">이름은 메뉴 재료에서 관리합니다</p>
              </div>
              
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  현재 재고량
                </label>
                <input
                  type="number"
                  min="0"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentItem.count}
                  onChange={(e) => setCurrentItem({...currentItem, count: parseInt(e.target.value)})}
                />
              </div>
              
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  최대 재고량
                </label>
                <input
                  type="number"
                  min="0"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentItem.max_count}
                  onChange={(e) => setCurrentItem({...currentItem, max_count: parseInt(e.target.value)})}
                />
              </div>
            </div>
            
            <div className="border-t px-6 py-4 flex justify-end">
              <button
                onClick={() => setIsModalOpen(false)}
                className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
              >
                취소
              </button>
              <button
                onClick={handleSaveItem}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 flex items-center"
              >
                <Save size={16} className="mr-2" />
                저장
              </button>
            </div>
          </div>
        </div>
      )}

      {/* 삭제 확인 모달 */}
      {isDeleteModalOpen && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
            <div className="px-6 py-4">
              <div className="text-center">
                <AlertTriangle className="h-12 w-12 text-red-500 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">
                  {deleteType === 'ingredient' ? '재료 삭제' : '재고 항목 삭제'}
                </h3>
                <p className="text-sm text-gray-500">
                  '{itemToDelete?.name}'을(를) 정말 삭제하시겠습니까? 이 작업은 되돌릴 수 없습니다.
                  {deleteType === 'ingredient' && ' 이 재료에 연결된 재고 항목도 함께 삭제됩니다.'}
                </p>
                
                {error && (
                  <div className="mt-4 p-2 bg-red-100 border border-red-400 text-red-700 rounded text-sm">
                    {error}
                  </div>
                )}
              </div>
            </div>
            
            <div className="border-t px-6 py-4 flex justify-center space-x-3">
              <button
                onClick={() => closeDeleteModal()}
                className="px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
              >
                취소
              </button>
              <button
                onClick={handleDeleteItem}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-red-600 hover:bg-red-700 flex items-center"
              >
                <Trash2 size={16} className="mr-2" />
                삭제
              </button>
            </div>
          </div>
        </div>
      )}

      {/* 재료 목록 모달 */}
      {isIngredientListModal && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl max-w-3xl w-full mx-4">
            <div className="flex justify-between items-center border-b px-6 py-4">
              <h3 className="text-lg font-medium text-gray-900">
                재료 관리
              </h3>
              <button
                onClick={() => closeIngredientListModal()}
                className="text-gray-400 hover:text-gray-500"
              >
                <X size={20} />
              </button>
            </div>
            
            <div className="p-6">
              {error && (
                <div className="mb-4 p-2 bg-red-100 border border-red-400 text-red-700 rounded">
                  {error}
                </div>
              )}
              
              <div className="flex justify-between mb-4">
                <div className="relative flex-1 max-w-md">
                  <input
                    type="text"
                    placeholder="재료 검색..."
                    value={ingredientSearchTerm}
                    onChange={(e) => setIngredientSearchTerm(e.target.value)}
                    className="w-full pl-10 pr-4 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  />
                  <div className="absolute inset-y-0 left-0 flex items-center pl-3">
                    <Search size={18} className="text-gray-400" />
                  </div>
                </div>
                
                <button
                  onClick={() => {
                    handleAddNewIngredient();
                    setError(null);
                  }}
                  className="ml-2 flex items-center px-4 py-2 border border-transparent shadow-sm text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700"
                >
                  <PlusCircle size={16} className="mr-2" />
                  새 재료 추가
                </button>
              </div>
              
              {filteredIngredients.length === 0 ? (
                <div className="text-center py-8">
                  <Package className="h-12 w-12 text-gray-400 mx-auto" />
                  <h3 className="mt-2 text-sm font-medium text-gray-900">재료 없음</h3>
                  <p className="mt-1 text-sm text-gray-500">
                    {ingredientSearchTerm ? '검색 조건에 맞는 재료가 없습니다.' : '등록된 재료가 없습니다.'}
                  </p>
                </div>
              ) : (
                <div className="overflow-y-auto max-h-96">
                  <table className="min-w-full divide-y divide-gray-200">
                    <thead className="bg-gray-50">
                      <tr>
                        <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          재료명
                        </th>
                        <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          연결된 메뉴
                        </th>
                        <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          필요 수량
                        </th>
                        <th className="px-6 py-3 text-right text-xs font-medium text-gray-500 uppercase tracking-wider">
                          작업
                        </th>
                      </tr>
                    </thead>
                    <tbody className="bg-white divide-y divide-gray-200">
                      {filteredIngredients.map((ingredient) => {
                        // 연결된 메뉴 찾기
                        const menuItem = menuItems.find(item => item.id === ingredient.menu_item_id);
                        
                        return (
                          <tr key={ingredient.id} className="hover:bg-gray-50">
                            <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">
                              {ingredient.name}
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                              {menuItem ? menuItem.name : '-'}
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                              {ingredient.quantity_required || 1}
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-right text-sm font-medium">
                              <div className="flex space-x-2 justify-end">
                                <button
                                  onClick={() => handleEditIngredient(ingredient)}
                                  className="text-indigo-600 hover:text-indigo-900 focus:outline-none"
                                  title="재료 수정"
                                >
                                  <Edit size={18} />
                                </button>
                                <button
                                  onClick={() => {
                                    handleDeleteConfirm(ingredient, 'ingredient');
                                    closeIngredientListModal();
                                  }}
                                  className="text-red-600 hover:text-red-900 focus:outline-none"
                                  title="재료 삭제"
                                >
                                  <Trash2 size={18} />
                                </button>
                              </div>
                            </td>
                          </tr>
                        );
                      })}
                    </tbody>
                  </table>
                </div>
              )}
            </div>
            
            <div className="border-t px-6 py-4 flex justify-end">
              <button
                onClick={() => closeIngredientListModal()}
                className="px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
              >
                닫기
              </button>
            </div>
          </div>
        </div>
      )}

      {/* 재료 추가/수정 모달 */}
      {isIngredientModalOpen && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
            <div className="flex justify-between items-center border-b px-6 py-4">
              <h3 className="text-lg font-medium text-gray-900">
                {currentItem.id ? '재료 수정' : '새 재료 추가'}
              </h3>
              <button
                onClick={() => closeIngredientModal()}
                className="text-gray-400 hover:text-gray-500"
              >
                <X size={20} />
              </button>
            </div>
            
            <div className="px-6 py-4">
              {/* 재료 이름 */}
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  재료 이름
                </label>
                <input
                  type="text"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentItem.name}
                  onChange={(e) => setCurrentItem({...currentItem, name: e.target.value})}
                  placeholder="재료 이름 입력"
                />
                <p className="mt-1 text-xs text-gray-500">
                  이미 존재하는 재료 이름은 사용할 수 없습니다
                </p>
              </div>
              
              {/* 연결 메뉴 */}
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  연결 메뉴 <span className="text-red-500">*</span>
                </label>
                <select
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentItem.menu_item_id}
                  onChange={(e) => setCurrentItem({...currentItem, menu_item_id: e.target.value})}
                >
                  <option value="">메뉴 선택</option>
                  {menuItems.map((item) => (
                    <option key={`menu-${item.id}`} value={item.id}>
                      {item.name}
                    </option>
                  ))}
                </select>
              </div>
              
              {/* 필요 수량 */}
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  필요 수량
                </label>
                <input
                  type="number"
                  min="1"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentItem.quantity_required}
                  onChange={(e) => setCurrentItem({...currentItem, quantity_required: parseInt(e.target.value) || 1})}
                />
                <p className="mt-1 text-xs text-gray-500">
                  메뉴 1개당 필요한 재료의 수량을 입력하세요
                </p>
              </div>
              
              {error && (
                <div className="mb-4 p-2 bg-red-100 border border-red-400 text-red-700 rounded">
                  {error}
                </div>
              )}
              
              <div className="flex justify-between border-t pt-4 mt-2">
                <p className="text-sm text-gray-500">
                  이 재료가 추가되면 자동으로 재고 항목도 생성됩니다.
                </p>
              </div>
            </div>
            
            <div className="border-t px-6 py-4 flex justify-end">
              <button
                onClick={() => closeIngredientModal()}
                className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
              >
                취소
              </button>
              <button
                onClick={handleSaveIngredient}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 flex items-center"
              >
                <Save size={16} className="mr-2" />
                저장
              </button>
            </div>
          </div>
        </div>
      )}
    </Layout>
  );
};

export default InventoryPage; 