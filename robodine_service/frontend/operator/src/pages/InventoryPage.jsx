import React, { useState, useEffect } from 'react';
import { 
  Database, AlertTriangle, Search, PlusCircle, 
  Edit, Trash2, Download, Filter, RefreshCw
} from 'react-feather';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';

const InventoryPage = () => {
  const [inventory, setInventory] = useState([]);
  const [filteredInventory, setFilteredInventory] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [searchTerm, setSearchTerm] = useState('');
  const [categoryFilter, setCategoryFilter] = useState('all');
  const [sortConfig, setSortConfig] = useState({ key: 'name', direction: 'ascending' });
  const { apiCall } = useAuth();

  const categories = ['음료', '메인 요리', '사이드 메뉴', '디저트', '소스', '기타'];

  useEffect(() => {
    fetchInventory();
  }, [apiCall]);

  useEffect(() => {
    // 검색어와 카테고리 필터 적용
    let result = [...inventory];
    
    if (searchTerm) {
      result = result.filter(item => 
        item.name.toLowerCase().includes(searchTerm.toLowerCase()) ||
        item.description.toLowerCase().includes(searchTerm.toLowerCase()) ||
        item.category.toLowerCase().includes(searchTerm.toLowerCase())
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

  const fetchInventory = async () => {
    setIsLoading(true);
    setError(null);
    
    try {
      const data = await apiCall('/api/inventory');
      setInventory(data);
    } catch (err) {
      console.error('Failed to load inventory:', err);
      setError('재고 정보를 불러올 수 없습니다');
    } finally {
      setIsLoading(false);
    }
  };

  const handleSort = (key) => {
    let direction = 'ascending';
    if (sortConfig.key === key && sortConfig.direction === 'ascending') {
      direction = 'descending';
    }
    setSortConfig({ key, direction });
  };

  const getStockStatusClass = (stock, threshold) => {
    const percentage = (stock / threshold) * 100;
    if (percentage <= 20) return 'bg-red-100 text-red-800';
    if (percentage <= 50) return 'bg-yellow-100 text-yellow-800';
    return 'bg-green-100 text-green-800';
  };

  const handleAddItem = () => {
    // 구현 예정
    alert('재고 추가 기능은 개발 중입니다.');
  };

  const handleEditItem = (item) => {
    // 구현 예정
    alert(`${item.name} 아이템 수정 기능은 개발 중입니다.`);
  };

  const handleDeleteItem = async (id) => {
    if (!window.confirm('정말로 이 아이템을 삭제하시겠습니까?')) {
      return;
    }
    
    try {
      await apiCall(`/api/inventory/${id}`, { method: 'DELETE' });
      setInventory(inventory.filter(item => item.id !== id));
    } catch (error) {
      console.error(`Failed to delete inventory item ${id}:`, error);
      alert('아이템 삭제 중 오류가 발생했습니다');
    }
  };

  const exportToCSV = () => {
    // 구현 예정
    alert('CSV 내보내기 기능은 개발 중입니다.');
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
                      onClick={() => handleSort('stock')}
                      className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer hover:bg-gray-100"
                    >
                      재고량
                      {sortConfig.key === 'stock' && (
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
                    <th
                      onClick={() => handleSort('updated_at')}
                      className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer hover:bg-gray-100"
                    >
                      최종 업데이트
                      {sortConfig.key === 'updated_at' && (
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
                  {filteredInventory.length > 0 ? (
                    filteredInventory.map((item) => (
                      <tr key={item.id} className="hover:bg-gray-50">
                        <td className="px-6 py-4 whitespace-nowrap">
                          <div className="font-medium text-gray-900">{item.name}</div>
                          <div className="text-sm text-gray-500">ID: {item.id}</div>
                        </td>
                        <td className="px-6 py-4">
                          <div className="text-sm text-gray-900 max-w-xs truncate">{item.description}</div>
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap">
                          <span className="px-2 py-1 inline-flex text-xs leading-5 font-semibold rounded-full bg-blue-100 text-blue-800">
                            {item.category}
                          </span>
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap">
                          <div className="flex items-center">
                            <div className="w-16 bg-gray-200 rounded-full h-2.5 mr-2">
                              <div 
                                className={`h-2.5 rounded-full ${
                                  item.stock < (item.threshold * 0.2) ? 'bg-red-500' :
                                  item.stock < (item.threshold * 0.5) ? 'bg-yellow-500' :
                                  'bg-green-500'
                                }`}
                                style={{ width: `${Math.min(100, (item.stock / item.threshold) * 100)}%` }}
                              ></div>
                            </div>
                            <span className={`px-2 py-1 text-xs rounded-full ${getStockStatusClass(item.stock, item.threshold)}`}>
                              {item.stock} / {item.threshold}
                            </span>
                          </div>
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                          {new Intl.NumberFormat('ko-KR', { style: 'currency', currency: 'KRW' }).format(item.price)}
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                          {new Date(item.updated_at).toLocaleString('ko-KR')}
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-right text-sm font-medium">
                          <div className="flex space-x-2 justify-end">
                            <button
                              onClick={() => handleEditItem(item)}
                              className="text-blue-600 hover:text-blue-900"
                              title="수정"
                            >
                              <Edit size={16} />
                            </button>
                            <button
                              onClick={() => handleDeleteItem(item.id)}
                              className="text-red-600 hover:text-red-900"
                              title="삭제"
                            >
                              <Trash2 size={16} />
                            </button>
                          </div>
                        </td>
                      </tr>
                    ))
                  ) : (
                    <tr>
                      <td colSpan="7" className="px-6 py-10 text-center text-gray-500">
                        {searchTerm || categoryFilter !== 'all' ? (
                          <>검색 결과가 없습니다. 다른 검색어나 필터를 사용해보세요.</>
                        ) : (
                          <>재고 항목이 없습니다. '재고 추가' 버튼을 눌러 항목을 추가해보세요.</>
                        )}
                      </td>
                    </tr>
                  )}
                </tbody>
              </table>
            </div>
          )}
        </div>
      </div>
    </Layout>
  );
};

export default InventoryPage; 