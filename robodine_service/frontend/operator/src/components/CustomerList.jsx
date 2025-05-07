import React, { useState, useMemo } from 'react';
import { Users, Plus, Edit, Trash2, User } from 'react-feather';

const CustomerList = ({ 
  customers, 
  assignments, 
  tables, 
  onCustomerSelect, 
  selectedCustomer, 
  onAddCustomer,
  onEditCustomer,
  onDeleteCustomer 
}) => {
  const [sortConfig, setSortConfig] = useState({ key: 'timestamp', direction: 'desc' });
  
  // 배정 정보를 기준으로 테이블 번호 가져오기
  const getAssignedTable = (customerId) => {
    // 해당 고객의 배정 정보 찾기
    const assignment = assignments.find(a => a.customer_id === customerId);
    if (!assignment) return null;
    
    // 테이블 ID를 이용해 테이블 정보 찾기
    const tableId = assignment.table_id;
    
    // 테이블 ID로 표시 (테이블 객체에서 table_number가 없으면 ID 사용)
    return tableId;
  };
  
  // 고객 목록 정렬
  const sortedCustomers = useMemo(() => {
    let sortableCustomers = [...customers];
    if (sortConfig.key) {
      sortableCustomers.sort((a, b) => {
        if (a[sortConfig.key] < b[sortConfig.key]) {
          return sortConfig.direction === 'asc' ? -1 : 1;
        }
        if (a[sortConfig.key] > b[sortConfig.key]) {
          return sortConfig.direction === 'asc' ? 1 : -1;
        }
        return 0;
      });
    }
    return sortableCustomers;
  }, [customers, sortConfig]);
  
  // 정렬 요청 처리
  const requestSort = (key) => {
    let direction = 'asc';
    if (sortConfig.key === key && sortConfig.direction === 'asc') {
      direction = 'desc';
    }
    setSortConfig({ key, direction });
  };
  
  // 정렬 방향 표시 헬퍼 함수
  const getSortDirectionIndicator = (columnName) => {
    if (sortConfig.key === columnName) {
      return sortConfig.direction === 'asc' ? '↑' : '↓';
    }
    return '';
  };
  
  // 포맷된 날짜 문자열 생성
  const formatDate = (timestamp) => {
    const date = new Date(timestamp);
    return `${date.getHours().toString().padStart(2, '0')}:${date.getMinutes().toString().padStart(2, '0')}:${date.getSeconds().toString().padStart(2, '0')}`;
  };

  // 디버깅 용도로 할당 데이터 출력
  // console.log("Assignments in CustomerList:", assignments);
  // console.log("Customers in CustomerList:", customers);

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
      
      <div className="overflow-y-auto" style={{ maxHeight: '480px' }}>
        <table className="min-w-full divide-y divide-gray-200">
          <thead className="bg-gray-50 sticky top-0">
            <tr>
              <th 
                scope="col" 
                className="px-3 py-2 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer"
                onClick={() => requestSort('id')}
              >
                ID {getSortDirectionIndicator('id')}
              </th>
              <th 
                scope="col" 
                className="px-3 py-2 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer"
                onClick={() => requestSort('count')}
              >
                인원 {getSortDirectionIndicator('count')}
              </th>
              <th 
                scope="col" 
                className="px-3 py-2 text-left text-xs font-medium text-gray-500 uppercase tracking-wider cursor-pointer"
                onClick={() => requestSort('timestamp')}
              >
                입장시간 {getSortDirectionIndicator('timestamp')}
              </th>
              <th 
                scope="col" 
                className="px-3 py-2 text-left text-xs font-medium text-gray-500 uppercase tracking-wider"
              >
                배정테이블
              </th>
              <th 
                scope="col" 
                className="px-3 py-2 text-right text-xs font-medium text-gray-500 uppercase tracking-wider"
              >
                Actions
              </th>
            </tr>
          </thead>
          <tbody className="bg-white divide-y divide-gray-200">
            {sortedCustomers.length > 0 ? (
              sortedCustomers.map((customer) => {
                const assignedTable = getAssignedTable(customer.id);
                const isSelected = selectedCustomer && selectedCustomer.id === customer.id;
                
                return (
                  <tr 
                    key={customer.id} 
                    className={`hover:bg-gray-50 cursor-pointer ${isSelected ? 'bg-blue-50' : ''}`}
                    onClick={() => onCustomerSelect(customer)}
                  >
                    <td className="px-3 py-2 whitespace-nowrap text-sm font-medium text-gray-900">
                      {customer.id}
                    </td>
                    <td className="px-3 py-2 whitespace-nowrap text-sm text-gray-500">
                      {customer.count}명
                    </td>
                    <td className="px-3 py-2 whitespace-nowrap text-sm text-gray-500">
                      {formatDate(customer.timestamp)}
                    </td>
                    <td className="px-3 py-2 whitespace-nowrap text-sm text-gray-500">
                      {assignedTable ? (
                        <span className="px-3 py-1 text-xs font-bold rounded-full bg-blue-100 text-blue-800 inline-block min-w-[80px] text-center">
                          테이블 {assignedTable}
                        </span>
                      ) : (
                        <span className="text-gray-400 px-3 py-1 text-xs rounded-full bg-gray-100 inline-block min-w-[80px] text-center">
                          미배정
                        </span>
                      )}
                    </td>
                    <td className="px-3 py-2 whitespace-nowrap text-right text-sm font-medium">
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          onEditCustomer(customer);
                        }}
                        className="text-blue-600 hover:text-blue-900 mr-2"
                      >
                        <Edit size={16} />
                      </button>
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          onDeleteCustomer(customer.id);
                        }}
                        className="text-red-600 hover:text-red-900"
                      >
                        <Trash2 size={16} />
                      </button>
                    </td>
                  </tr>
                );
              })
            ) : (
              <tr>
                <td colSpan="5" className="px-3 py-8 text-center text-gray-500">
                  <User size={32} className="mx-auto mb-2 text-gray-300" />
                  <p>등록된 고객이 없습니다.</p>
                </td>
              </tr>
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default CustomerList; 