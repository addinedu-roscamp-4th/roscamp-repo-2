import React, { useState, useEffect } from 'react';
import { 
  Users, Table, AlertTriangle, Plus, Edit, 
  Trash2, User, Info, MapPin, Menu, ArrowRight
} from 'react-feather';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';

const CustomerList = ({ customers, onSelectCustomer, onAddCustomer, onEditCustomer, onDeleteCustomer }) => {
  return (
    <div className="w-full md:w-1/3 bg-white rounded-lg shadow">
      <div className="p-4 border-b flex justify-between items-center">
        <h2 className="text-lg font-semibold flex items-center">
          <Users size={20} className="mr-2 text-blue-600" />
          고객 목록
        </h2>
        <button 
          onClick={onAddCustomer}
          className="p-1 rounded-full bg-blue-50 text-blue-600 hover:bg-blue-100"
          title="고객 추가"
        >
          <Plus size={20} />
        </button>
      </div>
      
      <div className="h-[540px] overflow-y-auto">
        {customers && customers.length > 0 ? (
          <ul className="divide-y divide-gray-200">
            {customers.map(customer => (
              <li 
                key={customer.id}
                className="hover:bg-gray-50 cursor-pointer"
                onClick={() => onSelectCustomer(customer)}
              >
                <div className="p-4">
                  <div className="flex justify-between items-start">
                    <div>
                      <h3 className="font-medium">{customer.name}</h3>
                      <p className="text-sm text-gray-500">{customer.phone_number || customer.phoneNumber}</p>
                      {customer.group_size && (
                        <div className="flex items-center mt-1 text-sm text-gray-500">
                          <Users size={14} className="mr-1" />
                          {customer.group_size}명
                        </div>
                      )}
                    </div>
                    <div className="flex space-x-1">
                      <button 
                        onClick={e => { e.stopPropagation(); onEditCustomer(customer); }}
                        className="p-1 text-gray-400 hover:text-blue-600"
                        title="고객 정보 수정"
                      >
                        <Edit size={16} />
                      </button>
                      <button 
                        onClick={e => { e.stopPropagation(); onDeleteCustomer(customer.id); }}
                        className="p-1 text-gray-400 hover:text-red-600"
                        title="고객 정보 삭제"
                      >
                        <Trash2 size={16} />
                      </button>
                    </div>
                  </div>
                  
                  {customer.table_number && (
                    <div className="mt-2 flex items-center">
                      <div className="px-2 py-1 bg-blue-100 text-blue-800 rounded text-xs">
                        테이블 {customer.table_number}
                      </div>
                      {customer.waiting_time && (
                        <div className="ml-2 text-xs text-gray-500">
                          대기 시간: {customer.waiting_time}분
                        </div>
                      )}
                    </div>
                  )}
                </div>
              </li>
            ))}
          </ul>
        ) : (
          <div className="p-8 text-center text-gray-500">
            <User size={36} className="mx-auto mb-3 opacity-25" />
            <p>고객 정보가 없습니다</p>
          </div>
        )}
      </div>
    </div>
  );
};

const TableSimulator = ({ tables, selectedCustomer, onAssignTable }) => {
  // 테이블 상태에 따른 색상
  const getTableColor = (status) => {
    switch (status) {
      case 'OCCUPIED':
        return 'bg-red-500 text-white';
      case 'AVAILABLE':
        return 'bg-green-500 text-white';
      case 'RESERVED':
        return 'bg-yellow-500 text-white';
      default:
        return 'bg-gray-300 text-gray-700';
    }
  };

  // 테이블 클릭 핸들러 - 선택된 고객에게 테이블 할당
  const handleTableClick = (tableId, tableNumber) => {
    if (selectedCustomer) {
      onAssignTable(selectedCustomer.id, tableId);
    } else {
      alert('먼저 고객을 선택해주세요');
    }
  };

  return (
    <div className="w-full md:w-2/3 bg-white rounded-lg shadow ml-0 md:ml-4 mt-4 md:mt-0">
      <div className="p-4 border-b">
        <h2 className="text-lg font-semibold flex items-center">
          <Table size={20} className="mr-2 text-blue-600" />
          테이블 관리
        </h2>
      </div>
      
      <div className="p-4">
        {selectedCustomer && (
          <div className="mb-4 bg-blue-50 p-3 rounded-lg flex items-center justify-between">
            <div className="flex items-center">
              <User size={18} className="mr-2 text-blue-600" />
              <span><strong>{selectedCustomer.name}</strong> 님을 테이블에 배정하려면 테이블을 클릭하세요</span>
            </div>
            {selectedCustomer.group_size && (
              <div className="bg-blue-100 text-blue-800 text-xs px-2 py-1 rounded">
                인원: {selectedCustomer.group_size}명
              </div>
            )}
          </div>
        )}
        
        <div className="relative bg-gray-100 p-4 rounded-lg" style={{ height: '500px' }}>
          {/* 테이블 표시 영역 */}
          {tables && tables.map(table => (
            <div
              key={table.id}
              className={`absolute cursor-pointer ${getTableColor(table.status)} rounded-lg shadow-md flex flex-col items-center justify-center transition-all hover:shadow-lg`}
              style={{
                width: '80px',
                height: '80px',
                left: `${table.position.x}px`,
                top: `${table.position.y}px`,
                transform: 'translate(-50%, -50%)'
              }}
              onClick={() => handleTableClick(table.id, table.number)}
            >
              <Table size={20} />
              <span className="font-bold mt-1">{table.number}</span>
              <span className="text-xs">{table.capacity}인석</span>
            </div>
          ))}
          
          {/* 테이블이 없는 경우 안내 메시지 */}
          {(!tables || tables.length === 0) && (
            <div className="absolute inset-0 flex items-center justify-center text-gray-500">
              <div className="text-center">
                <MapPin size={36} className="mx-auto mb-3 opacity-25" />
                <p>테이블 정보가 없습니다</p>
              </div>
            </div>
          )}
          
          {/* 테이블 범례 */}
          <div className="absolute right-2 bottom-2 bg-white p-2 rounded-lg shadow-sm text-xs">
            <div className="flex items-center mb-1">
              <div className="w-3 h-3 rounded-full bg-green-500 mr-1"></div>
              <span>이용 가능</span>
            </div>
            <div className="flex items-center mb-1">
              <div className="w-3 h-3 rounded-full bg-red-500 mr-1"></div>
              <span>이용 중</span>
            </div>
            <div className="flex items-center">
              <div className="w-3 h-3 rounded-full bg-yellow-500 mr-1"></div>
              <span>예약됨</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

const CustomerPage = () => {
  const [customers, setCustomers] = useState([]);
  const [tables, setTables] = useState([]);
  const [selectedCustomer, setSelectedCustomer] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const { apiCall } = useAuth();

  useEffect(() => {
    const fetchData = async () => {
      setIsLoading(true);
      setError(null);
      
      try {
        // 고객 데이터 로드
        const customersData = await apiCall('/api/customers');
        setCustomers(customersData);
        
        // 테이블 데이터 로드
        const tablesData = await apiCall('/api/tables');
        setTables(tablesData);
      } catch (err) {
        console.error('Failed to load data:', err);
        setError('고객 또는 테이블 정보를 불러올 수 없습니다');
      } finally {
        setIsLoading(false);
      }
    };

    fetchData();
  }, [apiCall]);

  const handleSelectCustomer = (customer) => {
    setSelectedCustomer(customer);
  };

  const handleAddCustomer = () => {
    // 구현 예정
    alert('고객 추가 기능은 개발 중입니다.');
  };

  const handleEditCustomer = (customer) => {
    // 구현 예정
    alert(`${customer.name} 고객 정보 수정 기능은 개발 중입니다.`);
  };

  const handleDeleteCustomer = async (id) => {
    if (!window.confirm('정말로 이 고객 정보를 삭제하시겠습니까?')) {
      return;
    }
    
    try {
      await apiCall(`/api/customers/${id}`, { method: 'DELETE' });
      setCustomers(customers.filter(c => c.id !== id));
      if (selectedCustomer && selectedCustomer.id === id) {
        setSelectedCustomer(null);
      }
    } catch (error) {
      console.error(`Failed to delete customer ${id}:`, error);
      alert('고객 정보 삭제 중 오류가 발생했습니다.');
    }
  };

  const handleAssignTable = async (customerId, tableId) => {
    try {
      await apiCall(`/api/customers/${customerId}/assign-table`, {
        method: 'PUT',
        body: JSON.stringify({ table_id: tableId })
      });
      
      // 고객 데이터 업데이트
      const updatedCustomer = customers.find(c => c.id === customerId);
      const tableNumber = tables.find(t => t.id === tableId)?.number;
      
      setCustomers(customers.map(c => 
        c.id === customerId ? { ...c, table_number: tableNumber } : c
      ));
      
      // 테이블 상태 업데이트
      setTables(tables.map(t => 
        t.id === tableId ? { ...t, status: 'OCCUPIED' } : t
      ));
      
      setSelectedCustomer(null);
    } catch (error) {
      console.error(`Failed to assign table:`, error);
      alert('테이블 배정 중 오류가 발생했습니다');
    }
  };

  return (
    <Layout>
      <div className="container mx-auto p-4">
        <div className="flex justify-between items-center mb-6">
          <h1 className="text-2xl font-bold text-gray-800 flex items-center">
            <Users className="text-blue-600 mr-2" size={28} />
            고객·테이블 관리
          </h1>
          
          <div className="flex space-x-2">
            <button className="flex items-center px-3 py-1 bg-gray-100 rounded hover:bg-gray-200">
              <Menu size={16} className="mr-1" />
              메뉴 보기
            </button>
            <button className="flex items-center px-3 py-1 bg-blue-600 text-white rounded hover:bg-blue-700">
              주문하기
              <ArrowRight size={16} className="ml-1" />
            </button>
          </div>
        </div>

        {error && (
          <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6 flex items-center">
            <AlertTriangle className="mr-2" size={20} />
            <span>{error}</span>
          </div>
        )}

        {isLoading ? (
          <div className="flex items-center justify-center h-64">
            <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
          </div>
        ) : (
          <div className="flex flex-col md:flex-row gap-4">
            <CustomerList 
              customers={customers}
              onSelectCustomer={handleSelectCustomer}
              onAddCustomer={handleAddCustomer}
              onEditCustomer={handleEditCustomer}
              onDeleteCustomer={handleDeleteCustomer}
            />
            <TableSimulator 
              tables={tables}
              selectedCustomer={selectedCustomer}
              onAssignTable={handleAssignTable}
            />
          </div>
        )}
      </div>
    </Layout>
  );
};

export default CustomerPage; 