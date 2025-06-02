import React, { useState, useEffect, useCallback } from 'react';
import { 
  Coffee, AlertTriangle, Search, PlusCircle, 
  Edit, Trash2, Download, Filter, RefreshCw,
  X, Save, CheckSquare, Package
} from 'lucide-react';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useWebSockets } from '../contexts/WebSocketContext';

const MenuPage = () => {
  const [menuItems, setMenuItems] = useState([]);
  const [menuIngredients, setMenuIngredients] = useState([]);
  const [filteredMenuItems, setFilteredMenuItems] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [searchTerm, setSearchTerm] = useState('');
  const [sortConfig, setSortConfig] = useState({ key: 'name', direction: 'ascending' });
  const [isItemModalOpen, setIsItemModalOpen] = useState(false);
  const [isIngredientModalOpen, setIsIngredientModalOpen] = useState(false);
  const [currentMenuItem, setCurrentMenuItem] = useState(null);
  const [currentIngredient, setCurrentIngredient] = useState(null);
  const [isDeleteModalOpen, setIsDeleteModalOpen] = useState(false);
  const [itemToDelete, setItemToDelete] = useState(null);
  const [deleteType, setDeleteType] = useState('');
  const { apiCall } = useAuth();
  const { data, connected } = useWebSockets();

  const BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

  // 초기 데이터 로딩
  const fetchInitialData = useCallback(async () => {
    setIsLoading(true);
    setError(null);
    
    try {
      // 메뉴 항목 데이터 가져오기
      const menuItemsData = await apiCall('/api/menu/items');
      
      // 이미지 URL과 설명 필드가 null이면 빈 문자열로 변환
      const processedMenuItems = menuItemsData.map(item => ({
        ...item,
        image_url: item.image_url || '',
        description: item.description || ''
      }));
      
      setMenuItems(processedMenuItems);
      
      // 메뉴 재료 데이터 가져오기
      const menuIngredientsData = await apiCall('/api/menu/ingredients');
      setMenuIngredients(menuIngredientsData);
      
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
    if (data.menu) {
      const { items, ingredients } = data.menu;
      
      // 데이터 구조 확인 후 상태 업데이트
      if (Array.isArray(items)) {
        const processedItems = items.map(item => {
          // API에서 받은 형식으로 변환
          const id = item["MenuItem.id"] || item.id || 0;
          const name = item["MenuItem.name"] || item.name || '';
          
          // 가격은 숫자로 변환하여 보장
          let price = 0;
          if (item["MenuItem.price"] !== undefined) {
            price = Number(item["MenuItem.price"]);
          } else if (item.price !== undefined) {
            price = Number(item.price);
          }
          
          const prepare_time = item["MenuItem.prepare_time"] || item.prepare_time || 0;
          // null 값은 빈 문자열로 처리
          const image_url = (item["MenuItem.image_url"] === null ? '' : item["MenuItem.image_url"]) || 
                           (item.image_url === null ? '' : item.image_url) || '';
          const description = (item["MenuItem.description"] === null ? '' : item["MenuItem.description"]) || 
                             (item.description === null ? '' : item.description) || '';
          
          return {
            id,
            name,
            price,
            prepare_time,
            image_url,
            description
          };
        });
        
        setMenuItems(processedItems);
        setIsLoading(false);
      }
      
      if (Array.isArray(ingredients)) {
        const processedIngredients = ingredients.map(ingredient => {
          // API에서 받은 형식으로 변환
          return {
            id: ingredient["MenuIngredient.id"] || ingredient.id || 0,
            name: ingredient["MenuIngredient.name"] || ingredient.name || '',
            menu_item_id: ingredient["MenuIngredient.menu_item_id"] || ingredient.menu_item_id || 0,
            quantity_required: ingredient["MenuIngredient.quantity_required"] || ingredient.quantity_required || 1
          };
        });
        
        setMenuIngredients(processedIngredients);
        setIsLoading(false);
      }
    }
  }, [data.menu]);

  // 필터링 및 정렬 적용
  useEffect(() => {
    let result = [...menuItems];
    
    if (searchTerm) {
      result = result.filter(item => 
        item.name.toLowerCase().includes(searchTerm.toLowerCase())
      );
    }
    
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
    
    setFilteredMenuItems(result);
  }, [menuItems, searchTerm, sortConfig]);

  // 정렬 처리
  const handleSort = (key) => {
    let direction = 'ascending';
    if (sortConfig.key === key && sortConfig.direction === 'ascending') {
      direction = 'descending';
    }
    setSortConfig({ key, direction });
  };

  // 메뉴 항목 모달 닫기
  const closeItemModal = () => {
    setIsItemModalOpen(false);
    setError('');
  };

  // 재료 모달 닫기
  const closeIngredientModal = () => {
    setIsIngredientModalOpen(false);
    setError('');
  };

  // CSV 내보내기
  const exportToCSV = () => {
    // CSV 헤더 생성
    const headers = ['ID', '메뉴명', '가격', '준비시간', '재료'];
    
    // 데이터 행 생성
    const rows = filteredMenuItems.map(item => {
      const ingredients = menuIngredients
        .filter(ing => ing.menu_item_id === item.id)
        .map(ing => ing.name)
        .join(', ');
      
      return [
        item.id,
        item.name,
        item.price,
        item.prepare_time,
        ingredients
      ];
    });
    
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
    link.setAttribute('download', `메뉴목록_${new Date().toISOString().slice(0, 10)}.csv`);
    document.body.appendChild(link);
    
    // 다운로드 실행
    link.click();
    
    // 정리
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  // 메뉴 항목 추가 모달 열기
  const handleAddMenuItem = () => {
    setCurrentMenuItem({
      name: '',
      price: '',
      prepare_time: '',
      image_url: '',
      description: ''
    });
    setError('');
    setIsItemModalOpen(true);
  };

  // 메뉴 항목 수정 모달 열기
  const handleEditMenuItem = (item) => {
    // 깊은 복사를 통해 현재 메뉴 아이템의 모든 속성을 복사
    setCurrentMenuItem({
      id: item.id,
      name: item.name || '',
      price: item.price || 0,
      prepare_time: item.prepare_time || 0,
      image_url: item.image_url || '',
      description: item.description || ''
    });
    setError('');
    setIsItemModalOpen(true);
  };

  // 메뉴 항목 삭제 모달 열기
  const handleDeleteConfirm = (item, type = 'menu') => {
    setItemToDelete(item);
    setDeleteType(type);
    setIsDeleteModalOpen(true);
  };

  // 메뉴 항목 삭제 실행
  const handleDeleteItem = async () => {
    if (!itemToDelete) return;
    
    try {
      if (deleteType === 'menu') {
        await apiCall(`/api/menu/items/${itemToDelete.id}`, 'DELETE');
      } else if (deleteType === 'ingredient') {
        await apiCall(`/api/menu/ingredients/${itemToDelete.id}`, 'DELETE');
      }
      
      setIsDeleteModalOpen(false);
      setItemToDelete(null);
      
      // 데이터 다시 불러오기
      fetchInitialData();
    } catch (error) {
      console.error(`삭제 실패 (ID: ${itemToDelete.id}):`, error);
      setError('항목 삭제 중 오류가 발생했습니다');
    }
  };

  // 메뉴 항목 저장 (추가/수정)
  const handleSaveMenuItem = async () => {
    try {
      // 필수 필드 검증
      if (!currentMenuItem.name || !currentMenuItem.name.trim()) {
        setError('메뉴 이름을 입력해 주세요');
        return;
      }
      
      // 가격이 숫자인지 확인
      const price = parseFloat(currentMenuItem.price);
      if (isNaN(price) || price < 0) {
        setError('유효한 가격을 입력해 주세요');
        return;
      }
      
      // 준비 시간이 숫자인지 확인
      const prepareTime = parseInt(currentMenuItem.prepare_time);
      if (isNaN(prepareTime) || prepareTime <= 0) {
        setError('유효한 준비 시간을 입력해 주세요');
        return;
      }
      
      const form = new FormData();
      form.append('name', currentMenuItem.name);
      form.append('price', currentMenuItem.price);
      form.append('prepare_time', currentMenuItem.prepare_time);
      form.append('description', currentMenuItem.description);
      if (currentMenuItem.file) {
        form.append('image', currentMenuItem.file);
      }
      const rawUrl = currentMenuItem.image_url || '';
      const fullUrl = rawUrl
        ? rawUrl.startsWith('http')
          ? rawUrl
          : `${BASE_URL}${rawUrl}`
        : '';
      form.append('image_url', fullUrl);
      for (let [key, value] of form.entries()) {
        console.log(key, value);
      }

      console.table(Array.from(form.entries()));

      
      let response;
      
      if (currentMenuItem.id) {
        // 기존 메뉴 수정
        response = await apiCall(`/api/menu/items/${currentMenuItem.id}`, 'PUT', form);
        
        // 웹소켓으로 데이터가 업데이트될 때까지 UI 미리 업데이트
        setMenuItems(prev => prev.map(item => 
          item.id === currentMenuItem.id ? { 
            ...item, 
            ...form, 
            id: currentMenuItem.id
          } : item
        ));
      } else {
        // 새 메뉴 추가
        try {
          response = await apiCall('/api/menu/items', 'POST', form);
        } catch (err) {
          // FastAPI가 보낸 에러 디테일을 보기
          const json = await err.response.json();
          console.error('Validation errors:', json);
        }        // 응답에서 새 메뉴 ID를 가져와 UI에 추가
        if (response && response.id) {
          const newItem = { 
            ...form, 
            id: response.id 
          };
          setMenuItems(prev => [...prev, newItem]);
        }
      }
      
      // 모달 닫기
      closeItemModal();
      setCurrentMenuItem(null);
      
    } catch (error) {
      console.error('메뉴 저장 실패:', error);
      setError('메뉴 저장 중 오류가 발생했습니다');
    }
  };

  // 재료 추가 모달 열기
  const handleAddIngredient = (menuItem) => {
    setCurrentIngredient({
      name: '',
      menu_item_id: menuItem.id,
      quantity_required: 1
    });
    setError('');
    setIsIngredientModalOpen(true);
  };

  // 재료 수정 모달 열기
  const handleEditIngredient = (ingredient) => {
    setCurrentIngredient(ingredient);
    setError('');
    setIsIngredientModalOpen(true);
  };

  // 재료 저장 (추가/수정)
  const handleSaveIngredient = async () => {
    try {
      // 필수 필드 검증
      if (!currentIngredient.name || !currentIngredient.name.trim()) {
        setError('재료 이름을 입력해 주세요');
        return;
      }

      if (!currentIngredient.menu_item_id) {
        setError('연결할 메뉴를 선택해 주세요');
        return;
      }
      
      const payload = {
        name: currentIngredient.name.trim(),
        menu_item_id: parseInt(currentIngredient.menu_item_id) || 0,
        quantity_required: parseInt(currentIngredient.quantity_required) || 1
      };
      
      // 메뉴 아이템 ID가 유효한지 확인
      if (payload.menu_item_id <= 0) {
        setError('유효한 메뉴를 선택해 주세요');
        return;
      }
      
      let response;
      
      if (currentIngredient.id) {
        // 기존 재료 수정
        response = await apiCall(`/api/menu/ingredients/${currentIngredient.id}`, 'PUT', payload);
        
        // 웹소켓으로 데이터가 업데이트될 때까지 UI 미리 업데이트
        setMenuIngredients(prev => prev.map(ingredient => 
          ingredient.id === currentIngredient.id ? { ...ingredient, ...payload, id: currentIngredient.id } : ingredient
        ));
      } else {
        // 새 재료 추가
        response = await apiCall('/api/menu/ingredients', 'POST', payload);
        
        // 응답에서 새 재료 ID를 가져와 UI에 추가
        if (response && response.id) {
          const newIngredient = { ...payload, id: response.id };
          setMenuIngredients(prev => [...prev, newIngredient]);
        }
      }
      
      // 모달 닫기
      closeIngredientModal();
      setCurrentIngredient(null);
      
    } catch (error) {
      console.error('재료 저장 실패:', error);
      setError('재료 저장 중 오류가 발생했습니다');
    }
  };

  return (
    <Layout>
      <div className="container mx-auto p-4">
        <div className="flex justify-between items-center mb-6">
          <h1 className="text-2xl font-bold text-gray-800 flex items-center">
            <Coffee className="text-blue-600 mr-2" size={28} />
            메뉴 관리
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
              onClick={handleAddMenuItem}
              className="flex items-center px-3 py-2 bg-blue-600 rounded-md shadow-sm text-sm font-medium text-white hover:bg-blue-700"
            >
              <PlusCircle size={16} className="mr-2" />
              메뉴 추가
            </button>
          </div>
        </div>

        {error && (
          <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6 flex items-center">
            <AlertTriangle className="mr-2" size={20} />
            <span>{error}</span>
          </div>
        )}
        
        <div className="bg-white rounded-lg shadow overflow-hidden mb-6">
          <div className="p-4 border-b flex flex-col md:flex-row justify-between space-y-2 md:space-y-0">
            <div className="relative flex-1 max-w-md">
              <input
                type="text"
                placeholder="메뉴 검색..."
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                className="w-full pl-10 pr-4 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
              />
              <div className="absolute inset-y-0 left-0 flex items-center pl-3">
                <Search size={18} className="text-gray-400" />
              </div>
            </div>
            
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
          ) : filteredMenuItems.length === 0 ? (
            <div className="text-center py-12">
              <Coffee className="mx-auto h-12 w-12 text-gray-400" />
              <h3 className="mt-2 text-sm font-medium text-gray-900">메뉴 없음</h3>
              <p className="mt-1 text-sm text-gray-500">
                {searchTerm 
                  ? '검색 조건에 맞는 메뉴가 없습니다.' 
                  : '등록된 메뉴가 없습니다.'}
              </p>
              <div className="mt-6">
                <button
                  onClick={handleAddMenuItem}
                  className="inline-flex items-center px-4 py-2 border border-transparent shadow-sm text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700"
                >
                  <PlusCircle className="-ml-1 mr-2 h-5 w-5" aria-hidden="true" />
                  메뉴 추가
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
                        <span className="ml-1" key={`name-sort-indicator-${sortConfig.direction}`}>
                          {sortConfig.direction === 'ascending' ? '↑' : '↓'}
                        </span>
                      )}
                    </th>
                    <th
                      onClick={() => handleSort('price')}
                      className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer hover:bg-gray-100"
                    >
                      가격
                      {sortConfig.key === 'price' && (
                        <span className="ml-1" key={`price-sort-indicator-${sortConfig.direction}`}>
                          {sortConfig.direction === 'ascending' ? '↑' : '↓'}
                        </span>
                      )}
                    </th>
                    <th 
                      onClick={() => handleSort('prepare_time')}
                      className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer hover:bg-gray-100"
                    >
                      준비시간
                      {sortConfig.key === 'prepare_time' && (
                        <span className="ml-1" key={`prepare-time-sort-indicator-${sortConfig.direction}`}>
                          {sortConfig.direction === 'ascending' ? '↑' : '↓'}
                        </span>
                      )}
                    </th>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      이미지
                    </th>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      설명
                    </th>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                      재료
                    </th>
                    <th className="px-6 py-3 text-right text-xs font-medium text-gray-500 uppercase tracking-wider">
                      작업
                    </th>
                  </tr>
                </thead>
                <tbody className="bg-white divide-y divide-gray-200">
                  {filteredMenuItems.map((item) => (
                    <tr key={`menu-item-${item.id}`} className="hover:bg-gray-50">
                      <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">
                        {item.name}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {item.price !== undefined ? item.price.toLocaleString() : '0'}원
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {item.prepare_time || 0}분
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {item.image_url ? (
                          <div className="h-12 w-12 relative">
                            <img 
                              src={item.image_url} 
                              alt={item.name} 
                              className="h-full w-full object-cover rounded"
                              onError={(e) => {
                                e.target.onerror = null;
                              }}
                            />
                          </div>
                        ) : (
                          <span className="text-xs text-gray-400">이미지 없음</span>
                        )}
                      </td>
                      <td className="px-6 py-4 text-sm text-gray-500 max-w-xs">
                        <div className="truncate">
                          {item.description || <span className="text-xs text-gray-400">설명 없음</span>}
                        </div>
                      </td>
                      <td className="px-6 py-4 text-sm text-gray-500">
                        <div className="flex flex-wrap gap-1">
                          {menuIngredients
                            .filter(ing => ing.menu_item_id === item.id)
                            .map(ing => (
                              <span 
                                key={`menu-${item.id}-ing-${ing.id}-${ing.name}`} 
                                className="px-2 py-1 text-xs bg-gray-100 rounded-full flex items-center"
                                title="클릭하여 재료 수정"
                                onClick={() => handleEditIngredient(ing)}
                              >
                                {ing.name}
                                <button 
                                  onClick={(e) => {
                                    e.stopPropagation();
                                    handleDeleteConfirm(ing, 'ingredient');
                                  }}
                                  className="ml-1 text-red-500 hover:text-red-700"
                                  title="재료 삭제"
                                >
                                  <X size={12} />
                                </button>
                              </span>
                            ))
                          }
                          <button
                            onClick={() => handleAddIngredient(item)}
                            className="px-2 py-1 text-xs text-blue-600 hover:bg-blue-50 rounded-full"
                            title="재료 추가"
                          >
                            <PlusCircle size={14} />
                          </button>
                        </div>
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-right text-sm font-medium">
                        <div className="flex space-x-2 justify-end">
                          <button
                            onClick={() => handleEditMenuItem(item)}
                            className="text-indigo-600 hover:text-indigo-900 focus:outline-none"
                            title="메뉴 수정"
                          >
                            <Edit size={18} />
                          </button>
                          <button
                            onClick={() => handleDeleteConfirm(item, 'menu')}
                            className="text-red-600 hover:text-red-900 focus:outline-none"
                            title="메뉴 삭제"
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

      {/* 메뉴 항목 추가/수정 모달 */}
      {isItemModalOpen && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
            <div className="flex justify-between items-center border-b px-6 py-4">
              <h3 className="text-lg font-medium text-gray-900">
                {currentMenuItem.id ? '메뉴 수정' : '새 메뉴 추가'}
              </h3>
              <button
                onClick={closeItemModal}
                className="text-gray-400 hover:text-gray-500"
              >
                <X size={20} />
              </button>
            </div>
            
            <div className="px-6 py-4">
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  메뉴 이름
                </label>
                <input
                  type="text"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentMenuItem.name}
                  onChange={(e) => setCurrentMenuItem({...currentMenuItem, name: e.target.value})}
                />
              </div>
              
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  가격
                </label>
                <input
                  type="number"
                  min="0"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentMenuItem.price}
                  onChange={(e) => setCurrentMenuItem({...currentMenuItem, price: e.target.value})}
                />
              </div>
              
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  준비 시간 (분)
                </label>
                <input
                  type="number"
                  min="1"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentMenuItem.prepare_time}
                  onChange={(e) => setCurrentMenuItem({...currentMenuItem, prepare_time: e.target.value})}
                />
              </div>

              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  설명
                </label>
                <textarea
                  rows="3"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentMenuItem.description}
                  onChange={(e) => setCurrentMenuItem({...currentMenuItem, description: e.target.value})}
                ></textarea>
              </div>

              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  이미지 URL
                </label>
                <input
                  type="text"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentMenuItem.image_url}
                  onChange={(e) => setCurrentMenuItem({...currentMenuItem, image_url: e.target.value})}
                />
              </div>

            {/* 파일 업로드 입력 */}
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-700 mb-1">
                이미지 파일
              </label>
              <input
                type="file"
                accept="image/*"
                onChange={(e) => {
                  const file = e.target.files[0];
                  setCurrentMenuItem(prev => ({ ...prev, file }));
                  // 미리보기
                  const reader = new FileReader();
                  reader.onload = () => setCurrentMenuItem(prev => ({ ...prev, preview: reader.result }));
                  reader.readAsDataURL(file);
                }}
              />
              {currentMenuItem.preview && (
                <img
                  src={currentMenuItem.preview}
                  alt="preview"
                  className="mt-2 h-24 w-24 object-cover rounded"
                />
              )}
            </div>
              
              {error && (
                <div className="mb-4 p-2 bg-red-100 border border-red-400 text-red-700 rounded">
                  {error}
                </div>
              )}
            </div>
            
            <div className="border-t px-6 py-4 flex justify-end">
              <button
                onClick={closeItemModal}
                className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
              >
                취소
              </button>
              <button
                onClick={handleSaveMenuItem}
                className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 flex items-center"
              >
                <Save size={16} className="mr-2" />
                저장
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
                {currentIngredient.id ? '재료 수정' : '새 재료 추가'}
              </h3>
              <button
                onClick={closeIngredientModal}
                className="text-gray-400 hover:text-gray-500"
              >
                <X size={20} />
              </button>
            </div>
            
            <div className="px-6 py-4">
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  재료 이름
                </label>
                <input
                  type="text"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentIngredient.name}
                  onChange={(e) => setCurrentIngredient({...currentIngredient, name: e.target.value})}
                />
              </div>
              
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  필요 수량
                </label>
                <input
                  type="number"
                  min="1"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentIngredient.quantity_required}
                  onChange={(e) => setCurrentIngredient({...currentIngredient, quantity_required: e.target.value})}
                />
              </div>
              
              {error && (
                <div className="mb-4 p-2 bg-red-100 border border-red-400 text-red-700 rounded">
                  {error}
                </div>
              )}
              
              <p className="text-sm text-gray-500 mb-2">
                이 재료가 추가되면 자동으로 재고 항목도 생성됩니다.
              </p>
            </div>
            
            <div className="border-t px-6 py-4 flex justify-end">
              <button
                onClick={closeIngredientModal}
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

      {/* 삭제 확인 모달 */}
      {isDeleteModalOpen && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
            <div className="px-6 py-4">
              <div className="text-center">
                <AlertTriangle className="h-12 w-12 text-red-500 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">
                  {deleteType === 'menu' ? '메뉴 삭제' : '재료 삭제'}
                </h3>
                <p className="text-sm text-gray-500">
                  '{itemToDelete?.name}'을(를) 정말 삭제하시겠습니까? 이 작업은 되돌릴 수 없습니다.
                  {deleteType === 'menu' && ' 이 메뉴의 모든 재료도 함께 삭제됩니다.'}
                </p>
              </div>
            </div>
            
            <div className="border-t px-6 py-4 flex justify-center space-x-3">
              <button
                onClick={() => setIsDeleteModalOpen(false)}
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
    </Layout>
  );
};

export default MenuPage; 