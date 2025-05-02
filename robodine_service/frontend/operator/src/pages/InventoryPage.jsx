import React, { useState, useEffect, useCallback } from 'react';
import { 
  Database, AlertTriangle, Search, PlusCircle, 
  Edit, Trash2, Download, Filter, RefreshCw,
  X, Save, CheckSquare
} from 'lucide-react';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';

// 웹소켓 연결 설정
const WS_BASE_URL = 'ws://127.0.0.1:8000/ws';

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
  const [categoryFilter, setCategoryFilter] = useState('all');
  const [sortConfig, setSortConfig] = useState({ key: 'name', direction: 'ascending' });
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [currentItem, setCurrentItem] = useState(null);
  const [isDeleteModalOpen, setIsDeleteModalOpen] = useState(false);
  const [itemToDelete, setItemToDelete] = useState(null);
  const { apiCall } = useAuth();

  const categories = ['음료', '메인 요리', '사이드 메뉴', '디저트', '소스', '기타'];
  const statusOptions = ['정상', '부족', '없음'];

  // 재고 데이터 로딩
  const fetchInventory = useCallback(async () => {
    setIsLoading(true);
    setError(null);
    
    try {
      // 재고 데이터 가져오기
      const inventoryData = await apiCall('/api/inventory');
      
      // 메뉴 항목 데이터 가져오기
      const menuItemsData = await apiCall('/api/menu/items');
      
      // 메뉴 재료 데이터 가져오기
      const menuIngredientsData = await apiCall('/api/menu/ingredients');
      
      // 데이터 설정
      setInventory(inventoryData);
      setMenuItems(menuItemsData);
      setMenuIngredients(menuIngredientsData);
      
      // 데이터 처리 및 가공
      const processedInventory = inventoryData.map(item => {
        // 연관된 메뉴 재료 찾기
        const ingredient = menuIngredientsData.find(ing => ing.id === item.ingredient_id);
        
        // 연관된 메뉴 항목 찾기
        let menuItem = null;
        if (ingredient && ingredient.menu_item_id) {
          menuItem = menuItemsData.find(menu => menu.id === ingredient.menu_item_id);
        }
        
        // 카테고리 결정 - 메뉴 항목에 따라 다르게
        let category = '기타';
        if (menuItem) {
          // 예시) 가격에 따라 카테고리 구분 (실제 로직은 다를 수 있음)
          if (menuItem.price >= 8000) category = '메인 요리';
          else if (menuItem.price >= 5000) category = '사이드 메뉴';
          else if (menuItem.price >= 3000) category = '음료';
          else category = '소스';
        }
        
        return {
          ...item,
          ingredient,
          menuItem,
          category,
          // 재고 임계값은 MenuIngredient.quantity_required의 3배로 가정
          threshold: ingredient ? ingredient.quantity_required * 3 : 10,
          price: menuItem ? menuItem.price : 0
        };
      });
      
      setInventory(processedInventory);
    } catch (err) {
      console.error('Failed to load inventory:', err);
      setError('재고 정보를 불러올 수 없습니다');
    } finally {
      setIsLoading(false);
    }
  }, [apiCall]);

  // 초기 데이터 로딩
  useEffect(() => {
    fetchInventory();
  }, [fetchInventory]);

  // 필터링 및 정렬 적용
  useEffect(() => {
    // 검색어와 카테고리 필터 적용
    let result = [...inventory];
    
    if (searchTerm) {
      result = result.filter(item => 
        (item.name && item.name.toLowerCase().includes(searchTerm.toLowerCase())) ||
        (item.description && item.description.toLowerCase().includes(searchTerm.toLowerCase())) ||
        (item.category && item.category.toLowerCase().includes(searchTerm.toLowerCase()))
      );
    }
    
    if (categoryFilter !== 'all') {
      result = result.filter(item => item.category === categoryFilter);
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
  }, [inventory, searchTerm, categoryFilter, sortConfig]);

  // 정렬 처리
  const handleSort = (key) => {
    let direction = 'ascending';
    if (sortConfig.key === key && sortConfig.direction === 'ascending') {
      direction = 'descending';
    }
    setSortConfig({ key, direction });
  };

  // 재고 상태에 따른 스타일 클래스
  const getStockStatusClass = (stock, threshold) => {
    const percentage = (stock / threshold) * 100;
    if (percentage <= 20) return 'bg-red-100 text-red-800';
    if (percentage <= 50) return 'bg-yellow-100 text-yellow-800';
    return 'bg-green-100 text-green-800';
  };

  // 재고 항목 추가 모달 열기
  const handleAddItem = () => {
    setCurrentItem({
      ingredient_id: 0,
      name: '',
      count: 0,
      status: 'NORMAL',
      category: '기타'
    });
    setIsModalOpen(true);
  };

  // 재고 항목 수정 모달 열기
  const handleEditItem = (item) => {
    setCurrentItem({
      id: item.id,
      ingredient_id: item.ingredient_id,
      name: item.name,
      count: item.count,
      status: item.status,
      category: item.category
    });
    setIsModalOpen(true);
  };

  // 재고 항목 삭제 모달 열기
  const handleDeleteConfirm = (item) => {
    setItemToDelete(item);
    setIsDeleteModalOpen(true);
  };

  // 재고 항목 삭제 실행
  const handleDeleteItem = async () => {
    if (!itemToDelete) return;
    
    try {
      await apiCall(`/api/inventory/${itemToDelete.id}`, 'DELETE');
      
      // 성공적으로 삭제 후 목록 갱신
      setInventory(inventory.filter(item => item.id !== itemToDelete.id));
      setIsDeleteModalOpen(false);
      setItemToDelete(null);
    } catch (error) {
      console.error(`Failed to delete inventory item ${itemToDelete.id}:`, error);
      setError('아이템 삭제 중 오류가 발생했습니다');
    }
  };

  // 재고 항목 저장 (추가/수정)
  const handleSaveItem = async () => {
    try {
      if (currentItem.id) {
        // 기존 항목 수정
        await apiCall(`/api/inventory/${currentItem.id}`, 'PUT', {
          ingredient_id: currentItem.ingredient_id,
          name: currentItem.name,
          count: currentItem.count,
          status: currentItem.status
        });
        
        // 목록 갱신
        setInventory(inventory.map(item => 
          item.id === currentItem.id ? { ...item, ...currentItem } : item
        ));
      } else {
        // 새 항목 추가
        const response = await apiCall('/api/inventory', 'POST', {
          ingredient_id: parseInt(currentItem.ingredient_id),
          name: currentItem.name,
          count: parseInt(currentItem.count),
          status: currentItem.status
        });
        
        // 응답에서 ID 가져오기
        const newItemId = response.id;
        
        // 목록에 새 항목 추가
        setInventory([...inventory, { ...currentItem, id: newItemId }]);
      }
      
      // 모달 닫기
      setIsModalOpen(false);
      setCurrentItem(null);
    } catch (error) {
      console.error('Failed to save inventory item:', error);
      setError('아이템 저장 중 오류가 발생했습니다');
    }
  };

  // CSV 내보내기
  const exportToCSV = () => {
    // CSV 헤더 생성
    const headers = ['ID', '이름', '카테고리', '재고량', '상태', '단가'];
    
    // 데이터 행 생성
    const rows = filteredInventory.map(item => [
      item.id,
      item.name,
      item.category,
      item.count,
      item.status,
      item.price ? `${item.price}원` : '-'
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
              onClick={handleAddItem}
              className="flex items-center px-3 py-2 bg-blue-600 rounded-md shadow-sm text-sm font-medium text-white hover:bg-blue-700"
            >
              <PlusCircle size={16} className="mr-2" />
              재고 추가
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
              <div className="relative">
                <select
                  value={categoryFilter}
                  onChange={(e) => setCategoryFilter(e.target.value)}
                  className="pl-10 pr-4 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500 appearance-none"
                >
                  <option value="all">모든 카테고리</option>
                  {categories.map(cat => (
                    <option key={cat} value={cat}>{cat}</option>
                  ))}
                </select>
                <div className="absolute inset-y-0 left-0 flex items-center pl-3">
                  <Filter size={18} className="text-gray-400" />
                </div>
              </div>
              
              <button 
                onClick={fetchInventory}
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
                {searchTerm || categoryFilter !== 'all' 
                  ? '검색 조건에 맞는 재고 항목이 없습니다.' 
                  : '등록된 재고 항목이 없습니다.'}
              </p>
              <div className="mt-6">
                <button
                  onClick={handleAddItem}
                  className="inline-flex items-center px-4 py-2 border border-transparent shadow-sm text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700"
                >
                  <PlusCircle className="-ml-1 mr-2 h-5 w-5" aria-hidden="true" />
                  재고 항목 추가
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
                      설명
                    </th>
                    <th
                      onClick={() => handleSort('category')}
                      className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer hover:bg-gray-100"
                    >
                      카테고리
                      {sortConfig.key === 'category' && (
                        <span className="ml-1">
                          {sortConfig.direction === 'ascending' ? '↑' : '↓'}
                        </span>
                      )}
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
                    <th
                      onClick={() => handleSort('price')}
                      className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer hover:bg-gray-100"
                    >
                      가격
                      {sortConfig.key === 'price' && (
                        <span className="ml-1">
                          {sortConfig.direction === 'ascending' ? '↑' : '↓'}
                        </span>
                      )}
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
                        {item.ingredient?.name || '-'}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {item.category}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap">
                        <span className={`px-2 inline-flex text-xs leading-5 font-semibold rounded-full ${getStockStatusClass(item.count, item.threshold)}`}>
                          {item.count} / {item.threshold}
                        </span>
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                        {item.price ? `${item.price.toLocaleString()}원` : '-'}
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

      {/* 재고 항목 추가/수정 모달 */}
      {isModalOpen && (
        <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
            <div className="flex justify-between items-center border-b px-6 py-4">
              <h3 className="text-lg font-medium text-gray-900">
                {currentItem.id ? '재고 항목 수정' : '새 재고 항목 추가'}
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
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentItem.name}
                  onChange={(e) => setCurrentItem({...currentItem, name: e.target.value})}
                />
              </div>
              
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  메뉴 재료 ID
                </label>
                <select
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentItem.ingredient_id}
                  onChange={(e) => setCurrentItem({...currentItem, ingredient_id: parseInt(e.target.value)})}
                >
                  <option value="">재료 선택</option>
                  {menuIngredients.map(ing => (
                    <option key={ing.id} value={ing.id}>
                      {ing.name || `재료 ID: ${ing.id}`}
                    </option>
                  ))}
                </select>
              </div>
              
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  카테고리
                </label>
                <select
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentItem.category}
                  onChange={(e) => setCurrentItem({...currentItem, category: e.target.value})}
                >
                  {categories.map(cat => (
                    <option key={cat} value={cat}>{cat}</option>
                  ))}
                </select>
              </div>
              
              <div className="mb-4">
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  재고량
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
                  상태
                </label>
                <select
                  className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                  value={currentItem.status}
                  onChange={(e) => setCurrentItem({...currentItem, status: e.target.value})}
                >
                  <option value="NORMAL">정상</option>
                  <option value="LOW">부족</option>
                  <option value="OUT_OF_STOCK">없음</option>
                </select>
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
                <h3 className="text-lg font-medium text-gray-900 mb-2">재고 항목 삭제</h3>
                <p className="text-sm text-gray-500">
                  '{itemToDelete?.name}'을(를) 정말 삭제하시겠습니까? 이 작업은 되돌릴 수 없습니다.
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

export default InventoryPage; 