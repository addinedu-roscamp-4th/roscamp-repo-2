import React, { useState, useEffect, useCallback, useMemo } from 'react';
import { 
  Users, Table, AlertTriangle, Plus, Edit, 
  Trash2, User, Info, MapPin, Menu, ArrowRight,
  X, Save, RefreshCw, Phone, UserPlus, ZoomIn, ZoomOut,
  HelpCircle
} from 'react-feather';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useWebSockets } from '../contexts/WebSocketContext';
import CustomerList from '../components/CustomerList';
import TableMap from '../components/TableMap';
import '../styles/CustomerPage.css';

// 웹소켓 토픽 리스트 (Context와 동일하게 유지)
const TOPICS = ['customers', 'tables'];

const CustomerPage = () => {
  const [selectedCustomer, setSelectedCustomer] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [isAddModalOpen, setIsAddModalOpen] = useState(false);
  const [isEditModalOpen, setIsEditModalOpen] = useState(false);
  const [isConfirmModalOpen, setIsConfirmModalOpen] = useState(false);
  const [confirmAction, setConfirmAction] = useState({ type: '', data: null });
  const [currentCustomer, setCurrentCustomer] = useState({ count: 1 });
  const [actionMessage, setActionMessage] = useState({ text: '', isError: false });
  const [lastUpdateTime, setLastUpdateTime] = useState(new Date());
  const { apiCall } = useAuth();
  const { data, errors, connected, refreshTopic } = useWebSockets();
  const { customers, tables } = data;
  const [processedCustomers, setProcessedCustomers] = useState([]);
  const [processedTables, setProcessedTables] = useState([]);
  const [processedAssignments, setProcessedAssignments] = useState([]);

  // 웹소켓 데이터 업데이트 시 로딩 해제
  useEffect(() => {
    if (data?.customers && data?.tables) {
      setIsLoading(false);  // 데이터가 완전히 로딩되었으면 로딩 상태를 false로 설정
      setLastUpdateTime(new Date());  // 데이터 업데이트 시간 갱신
    }
  }, [data]);

  // console.log('WebSocket Data:', data);

  // 테이블 배정 정보 처리
  const processedAssignmentsData = useMemo(() => {
    // 배정 정보 가져오기 (released_at이 null인 활성 배정만)
    const tableAssignments = data?.tables?.assignments && Array.isArray(data.tables.assignments)
      ? data.tables.assignments
          .filter(a => a['GroupAssignment.released_at'] === null)
          .map((a) => ({
            id: a['GroupAssignment.id'],
            table_id: a['GroupAssignment.table_id'],
            customer_id: a['GroupAssignment.customer_id'],
            timestamp: a['GroupAssignment.timestamp'],
          }))
      : [];
    
    const customerAssignments = data?.customers?.assignments && Array.isArray(data.customers.assignments)
      ? data.customers.assignments
          .filter(a => a['GroupAssignment.released_at'] === null)
          .map((a) => ({
            id: a['GroupAssignment.id'],
            table_id: a['GroupAssignment.table_id'],
            customer_id: a['GroupAssignment.customer_id'],
            timestamp: a['GroupAssignment.timestamp'],
          }))
      : [];
    
    // 중복 제거
    const merged = [...tableAssignments, ...customerAssignments];
    const deduplicated = merged.filter((assignment, index, self) =>
      index === self.findIndex((a) => a && a.id === assignment.id)
    );
    
    // console.log("테이블 배정 정보:", deduplicated);
    return deduplicated;
  }, [data]);

  // 고객 정보 처리
  const processedCustomersData = useMemo(() => {
    const base = Array.isArray(customers) ? customers : [];

    return base.map(c => {
      const customer = {
        id: c['Customer.id'],
        count: c['Customer.count'],
        timestamp: c['Customer.timestamp'],
      };

      // 배정된 테이블 정보 추가
      const assignment = processedAssignmentsData.find(a => a.customer_id === customer.id);
      if (assignment) {
        customer.table_id = assignment.table_id;
        customer.assignment_timestamp = assignment.timestamp;
      }

      return customer;
    });
  }, [customers, processedAssignmentsData]);

  // processedAssignmentsData가 업데이트되면 processedAssignments 상태 업데이트
  useEffect(() => {
    setProcessedAssignments(processedAssignmentsData);
  }, [processedAssignmentsData]);

  // 테이블 정보 처리
  const processedTablesData = useMemo(() => {
    // 테이블이 배열이 아닌 경우 빈 배열 반환
    if (!data.tables || !Array.isArray(data.tables.tables)) return [];

    const tables = data.tables.tables.map((t) => ({
      id: t['Table.id'],
      max_customer: t['Table.max_customer'],
      status: t['Table.status'],
      x: t['Table.x'],
      y: t['Table.y'],
      width: t['Table.width'],
      height: t['Table.height'],
    }));

    return tables.map((table) => {
      const hasAssignment = processedAssignmentsData.some((a) => a.table_id === table.id);
      return {
        ...table,
        status: hasAssignment ? 'OCCUPIED' : table.status || 'AVAILABLE',
        customer_id: hasAssignment ? processedAssignmentsData.find((a) => a.table_id === table.id).customer_id : null,
      };
    });
  }, [data, processedAssignmentsData]);

  // processedTablesData가 업데이트되면 processedTables 상태 업데이트
  useEffect(() => {
    setProcessedTables(processedTablesData);
  }, [processedTablesData]);

  // processedCustomersData가 업데이트되면 processedCustomers 상태 업데이트
  useEffect(() => {
    setProcessedCustomers(processedCustomersData);
  }, [processedCustomersData]);

  // console.log('Processed Customers:', processedCustomersData);
  // console.log('Processed Tables:', processedTablesData);
  // console.log('Processed Assignments:', processedAssignmentsData);

  const handleWebSocketUpdate = useCallback((newData) => {
    // 웹소켓으로 받은 데이터를 직접 처리하여 로컬 상태 업데이트
    if (newData.type === 'customer_update' && newData.customer) {
      const { customer } = newData;
      
      if (customer.action === 'add') {
        // 고객 추가
        setProcessedCustomers(prev => [customer, ...prev]);
      } 
      else if (customer.action === 'update') {
        // 고객 정보 업데이트
        setProcessedCustomers(prev => 
          prev.map(c => c.id === customer.id ? {...c, ...customer} : c)
        );
      } 
      else if (customer.action === 'delete') {
        // 고객 삭제
        setProcessedCustomers(prev => prev.filter(c => c.id !== customer.id));
        
        // 관련 배정 정보도 삭제
        setProcessedAssignments(prev => prev.filter(a => a.customer_id !== customer.id));
      }
    }
  
    if (newData.type === 'table_update' && newData.assignment) {
      const { assignment, action } = newData;
      
      if (action === 'assign') {
        // 테이블 배정 정보 추가
        setProcessedAssignments(prev => {
          // 이미 있는 배정이면 업데이트, 없으면 추가
          const exists = prev.some(a => a.table_id === assignment.table_id);
          if (exists) {
            return prev.map(a => a.table_id === assignment.table_id ? assignment : a);
          } else {
            return [...prev, assignment];
          }
        });
      }
      else if (action === 'release') {
        // 테이블 배정 해제
        setProcessedAssignments(prev => prev.filter(a => a.table_id !== assignment.table_id));
      }
    }
  }, []);

  // 웹소켓 업데이트 핸들러 등록
  useEffect(() => {
    // WebSocketContext에서 제공하는 메시지 핸들러가 있다면 등록
    if (typeof window !== 'undefined' && window.addEventListener) {
      window.addEventListener('websocket-message', (event) => {
        if (event.detail) {
          handleWebSocketUpdate(event.detail);
        }
      });
      
      return () => {
        window.removeEventListener('websocket-message', (event) => {
          if (event.detail) {
            handleWebSocketUpdate(event.detail);
          }
        });
      };
    }
  }, [handleWebSocketUpdate]);

  // 웹소켓 데이터 업데이트 시 로딩 해제
  useEffect(() => {
    if (data?.customers && data?.tables) {
      // 데이터가 모두 로딩되었으면 로딩 상태를 false로 설정
      setIsLoading(false);
      setLastUpdateTime(new Date());  // 데이터 업데이트 시간 갱신
    }
  }, [data]);  // data가 변경될 때마다 실행

  // 수동 새로고침 핸들러
  const handleRefreshData = useCallback(() => {
    setIsLoading(true);
    TOPICS.forEach(topic => refreshTopic(topic));
    setTimeout(() => setIsLoading(false), 1000);
  }, [refreshTopic]);

  // 알림 메시지 표시
  const showMessage = (text, isError = false) => {
    setActionMessage({ text, isError });
    setTimeout(() => setActionMessage({ text: '', isError: false }), 3000);
  };

  const handleApiResponse = useCallback((type, isSuccess, message) => {
    setActionMessage({
      text: isSuccess 
        ? `${message} 완료되었습니다.` 
        : `${message} 실패했습니다.`,
      isError: !isSuccess
    });
    setTimeout(() => { setActionMessage({ text: '', isError: false }); }, 3000);
  }, []);

  const handleSelectCustomer = (customer) => {
    if (selectedCustomer && selectedCustomer.id === customer.id) {
      setSelectedCustomer(null);
    } else {
      setSelectedCustomer(customer);
    }
  };

  const handleAddModalOpen = () => { setCurrentCustomer({ count: 1 }); setIsAddModalOpen(true); };
  const handleAddModalClose = () => setIsAddModalOpen(false);
  const handleEditModalOpen = (customer) => { setCurrentCustomer({ ...customer }); setIsEditModalOpen(true); };
  const handleEditModalClose = () => setIsEditModalOpen(false);
  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setCurrentCustomer(prev => ({
      ...prev,
      [name]: name === 'count' ? parseInt(value, 10) || 1 : value
    }));
  };

  const handleAddCustomer = async () => {
    try {
      setIsLoading(true);
      const response = await apiCall('/api/customers', 'POST', { count: currentCustomer.count });
      
      // API 응답이 정상이고 새 고객 ID가 반환된 경우
      if (response && response.id) {
        // 새 고객 객체 생성 (API 응답 형식에 맞춤)
        const newCustomer = {
          "Customer.id": response.id,
          "Customer.count": currentCustomer.count,
          "Customer.timestamp": new Date().toISOString(),
        };
        
        // 원본 데이터 업데이트 (data.customers)
        if (Array.isArray(data.customers)) {
          data.customers = [newCustomer, ...data.customers];
        } else {
          data.customers = [newCustomer];
        }
      }
      
      handleAddModalClose();
      handleApiResponse('add', true, '고객 추가가');
    } catch (error) {
      console.error('Failed to add customer:', error);
      handleApiResponse('add', false, '고객 추가가');
    } finally {
      setIsLoading(false);
    }
  };

  const handleEditCustomer = async () => {
    try {
      setIsLoading(true);
      await apiCall(`/api/customers/${currentCustomer.id}`, 'PUT', { count: currentCustomer.count });
      
      // 원본 데이터 업데이트
      if (Array.isArray(data.customers)) {
        data.customers = data.customers.map(c => 
          c['Customer.id'] === currentCustomer.id 
            ? { ...c, 'Customer.count': currentCustomer.count } 
            : c
        );
      }
      
      handleEditModalClose();
      setSelectedCustomer(null);
      handleApiResponse('edit', true, '고객 정보 수정이');
    } catch (error) {
      console.error('Failed to update customer:', error);
      handleApiResponse('edit', false, '고객 정보 수정이');
    } finally {
      setIsLoading(false);
    }
  };

  const handleDeleteCustomer = async (customerId) => {
    if (!window.confirm('해당 고객을 삭제하시겠습니까?')) return;

    try {
      setIsLoading(true);
      
      await apiCall(`/api/customers/${customerId}`, 'DELETE');
      
      // 선택된 고객이 삭제될 경우 선택 해제
      if (selectedCustomer && selectedCustomer.id === customerId) {
        setSelectedCustomer(null);
      }
      
      // 원본 데이터에서 고객 삭제
      if (Array.isArray(data.customers)) {
        data.customers = data.customers.filter(c => c['Customer.id'] !== customerId);
      }
      
      // 원본 데이터에서 해당 고객의 배정 정보 삭제
      if (data.tables && Array.isArray(data.tables.assignments)) {
        data.tables.assignments = data.tables.assignments.filter(
          a => a['GroupAssignment.customer_id'] !== customerId
        );
      }
      
      if (data.customers && Array.isArray(data.customers.assignments)) {
        data.customers.assignments = data.customers.assignments.filter(
          a => a['GroupAssignment.customer_id'] !== customerId
        );
      }
      
      handleApiResponse('delete', true, '고객 삭제가');
    } catch (error) {
      console.error('Failed to delete customer:', error);
      handleApiResponse('delete', false, '고객 삭제가');
    } finally {
      setIsLoading(false);
    }
  };

  const handleTableClick = (tableId) => {
    const table = processedTablesData.find(t => t.id === tableId);
    if (!table) return;
    
    // 테이블이 배정되어 있는지 확인
    const assignment = processedAssignmentsData.find(a => a.table_id === tableId);
    
    // 테이블이 이미 배정된 상태
    if (assignment || table.status === 'OCCUPIED') {
      // 배정 해제 프로세스 시작
      handleReleaseTable(tableId);
    } 
    // 테이블이 배정되지 않은 상태이고 고객이 선택되어 있음
    else if (selectedCustomer) {
      // 고객이 이미 다른 테이블에 배정되어 있는지 확인
      const existingAssignment = processedAssignmentsData.find(a => a.customer_id === selectedCustomer.id);
      // console.log('Existing Assignment:', existingAssignment);
      
      if (existingAssignment) {
        // 이미 다른 테이블에 배정되어 있으면 알림
        handleApiResponse('assign', false, `해당 고객은 이미 테이블 ${existingAssignment.table_id}번에 배정되어 있습니다`);
      } else {
        // 배정 프로세스 시작
        handleAssignTable(selectedCustomer.id, tableId);
      }
    } else {
      // 고객 선택 안내 메시지
      handleApiResponse('select', false, '먼저 고객을 선택한 후 테이블을 클릭하세요');
    }
  };

  const handleAssignTable = async (customerId, tableId) => {
    // 확인 모달 표시
    setConfirmAction({
      type: 'assign',
      data: { customerId, tableId }
    });
    setIsConfirmModalOpen(true);
  };

  const handleReleaseTable = async (tableId) => {
    // 확인 모달 표시
    setConfirmAction({
      type: 'release',
      data: { tableId }
    });
    setIsConfirmModalOpen(true);
  };

  const executeAssignTable = async (customerId, tableId) => {
    try {
      setIsLoading(true);
      
      // console.log(`Assigning customer ${customerId} to table ${tableId}`);
      
      // 테이블 배정 API 호출
      const response = await apiCall(`/api/tables/${tableId}/assign`, 'POST', {
        customer_id: customerId
      });

      // console.log("Assignment API response:", response);

      // API 응답 성공 시 로컬 데이터 업데이트
      if (response) {
        // 새로운 배정 정보 생성 (API 응답 형식에 맞춤)
        const newAssignment = {
          'GroupAssignment.id': Date.now(), // 임시 ID
          'GroupAssignment.table_id': tableId,
          'GroupAssignment.customer_id': customerId,
          'GroupAssignment.timestamp': new Date().toISOString(),
          'GroupAssignment.released_at': null
        };

        // console.log("New assignment data:", newAssignment);

        // 원본 데이터에 배정 정보 추가
        if (data.tables) {
          if (!data.tables.assignments) {
            data.tables.assignments = [];
          }
          data.tables.assignments.push(newAssignment);
        }
        
        if (data.customers) {
          if (!data.customers.assignments) {
            data.customers.assignments = [];
          }
          data.customers.assignments.push(newAssignment);
        }
        
        // 테이블 상태 업데이트
        if (data.tables && Array.isArray(data.tables.tables)) {
          data.tables.tables = data.tables.tables.map(t => 
            t['Table.id'] === tableId 
              ? { ...t, 'Table.status': 'OCCUPIED' } 
              : t
          );
        }
        
        // 선택된 고객 정보 초기화
        setSelectedCustomer(null);
        
        // 할당 정보를 기반으로 프로세스된 데이터 다시 계산하기 위해 설정
        setLastUpdateTime(new Date());
        
        handleApiResponse('assign', true, `테이블 ${tableId}번 배정이`);
      }
    } catch (error) {
      console.error(`Failed to assign table:`, error);
      handleApiResponse('assign', false, '테이블 배정이');
      
      // 수동으로 데이터 새로고침
      refreshTopic('tables');
      refreshTopic('customers');
    } finally {
      setIsLoading(false);
    }
  };

  const executeReleaseTable = async (tableId) => {
    try {
      setIsLoading(true);
      
      // console.log(`Releasing table ${tableId}`);
      
      // 테이블 배정 해제 API 호출
      const response = await apiCall(`/api/tables/${tableId}/release`, 'POST');
      
      // console.log("Release API response:", response);
      
      // API 응답 성공 시 로컬 데이터 업데이트
      if (response) {
        if (data.tables && Array.isArray(data.tables.assignments)) {
          // 배정 정보 삭제 대신 released_at 업데이트
          data.tables.assignments = data.tables.assignments.map(a => 
            a['GroupAssignment.table_id'] === tableId 
              ? { ...a, 'GroupAssignment.released_at': new Date().toISOString() } 
              : a
          );
        }
        
        if (data.customers && Array.isArray(data.customers.assignments)) {
          // 배정 정보 삭제 대신 released_at 업데이트
          data.customers.assignments = data.customers.assignments.map(a => 
            a['GroupAssignment.table_id'] === tableId 
              ? { ...a, 'GroupAssignment.released_at': new Date().toISOString() } 
              : a
          );
        }
        
        // 테이블 상태 업데이트
        if (data.tables && Array.isArray(data.tables.tables)) {
          data.tables.tables = data.tables.tables.map(t => 
            t['Table.id'] === tableId 
              ? { ...t, 'Table.status': 'AVAILABLE' } 
              : t
          );
        }
        
        // 할당 정보를 기반으로 프로세스된 데이터 다시 계산하기 위해 설정
        setLastUpdateTime(new Date());
        
        handleApiResponse('release', true, `테이블 ${tableId}번 배정 해제가`);
      }
    } catch (error) {
      console.error(`Failed to release table:`, error);
      handleApiResponse('release', false, '테이블 배정 해제가');
      
      // 수동으로 데이터 새로고침
      refreshTopic('tables');
      refreshTopic('customers');
    } finally {
      setIsLoading(false);
    }
  };

  const handleConfirmAction = () => {
    setIsConfirmModalOpen(false);
    
    if (confirmAction.type === 'assign') {
      const { customerId, tableId } = confirmAction.data;
      executeAssignTable(customerId, tableId);
    } else if (confirmAction.type === 'release') {
      const { tableId } = confirmAction.data;
      executeReleaseTable(tableId);
    }
  };

  const handleCancelAction = () => {
    setIsConfirmModalOpen(false);
    setConfirmAction({ type: '', data: null });
  };

  const formatDateTime = date =>
    date.toLocaleString('ko-KR', {
      year: 'numeric', month: 'long', day: 'numeric',
      weekday: 'long', hour: '2-digit', minute: '2-digit'
    });

    return (
      <Layout>
        <div className="container mx-auto p-4">
          <div className="flex justify-between items-center mb-6">
            <h1 className="text-2xl font-bold text-gray-800 flex items-center">
              <Users className="text-blue-600 mr-2" size={28} />
              고객·테이블 관리
            </h1>
            
            <div className="flex space-x-2 items-center">
              <p className="text-sm text-gray-500 mr-2">마지막 업데이트: {formatDateTime(lastUpdateTime)}</p>
              {(!connected.customers || !connected.tables) && <span className="text-xs bg-red-100 text-red-800 px-2 py-1 rounded-full mr-2">연결 끊김</span>}
              <button 
                onClick={handleRefreshData} 
                className="flex items-center px-3 py-1 bg-gray-100 rounded hover:bg-gray-200"
                disabled={isLoading}
              >
                <RefreshCw size={16} className={`mr-1 ${isLoading ? 'animate-spin' : ''}`} />
                새로고침
              </button>
              <button className="flex items-center px-3 py-1 bg-blue-600 text-white rounded hover:bg-blue-700">
                주문하기
                <ArrowRight size={16} className="ml-1" />
              </button>
            </div>
          </div>
  
          {/* 사용 안내 */}
          <div className="bg-blue-50 text-blue-700 px-4 py-3 rounded mb-6 flex items-start">
            <HelpCircle className="mr-2 mt-0.5 flex-shrink-0" size={20} />
            <div>
              <h3 className="font-medium">테이블 배정 방법</h3>
              <ul className="list-disc list-inside text-sm mt-1 space-y-1">
                <li>고객 목록에서 고객을 선택한 후 배정할 테이블을 클릭하세요</li>
                <li>이미 사용 중인 테이블(빨간색)을 클릭하면 배정을 해제할 수 있습니다</li>
                <li>배정된 테이블은 고객 정보 아래에 표시됩니다</li>
              </ul>
            </div>
          </div>
  
          {(errors.customers || errors.tables) && (
            <div className="bg-yellow-100 border-l-4 border-yellow-500 text-yellow-700 p-4 mb-4 rounded">
              <div className="flex">
                <AlertTriangle className="h-5 w-5 mr-2" />
                <p>데이터 가져오기 오류: {errors.customers || errors.tables}</p>
              </div>
            </div>
          )}
  
          {actionMessage.text && (
            <div className={`${actionMessage.isError ? 'bg-red-100 border-red-400 text-red-700' : 'bg-green-100 border-green-400 text-green-700'} border px-4 py-3 rounded mb-6 flex items-center`}>
              <Info className="mr-2" size={20} />
              <span>{actionMessage.text}</span>
            </div>
          )}
  
          {isLoading && !processedCustomersData.length ? (
            <div className="flex items-center justify-center h-64">
              <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
            </div>
          ) : (
            <div className="flex flex-col md:flex-row gap-4">
              <CustomerList 
                customers={processedCustomersData || []}
                onCustomerSelect={handleSelectCustomer}
                onAddCustomer={handleAddModalOpen}
                onEditCustomer={handleEditModalOpen}
                onDeleteCustomer={handleDeleteCustomer}
                assignments={processedAssignmentsData || []}
                tables={processedTablesData || []}
                selectedCustomer={selectedCustomer}
              />
              <TableMap 
                tables={processedTablesData || []}
                assignments={processedAssignmentsData || []}
                selectedCustomer={selectedCustomer}
                onTableClick={handleTableClick}
              />
            </div>
          )}
        </div>

        {/* 고객 추가 모달 */}
        {isAddModalOpen && (
          <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
            <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
              <div className="flex justify-between items-center border-b px-6 py-4">
                <h3 className="text-lg font-medium text-gray-900">
                  새 고객 등록
                </h3>
                <button
                  onClick={handleAddModalClose}
                  className="text-gray-400 hover:text-gray-500"
                >
                  <X size={20} />
                </button>
              </div>
              
              <div className="px-6 py-4">
                <div className="mb-4">
                  <label className="block text-sm font-medium text-gray-700 mb-1">
                    인원 수
                  </label>
                  <input
                    type="number"
                    name="count"
                    min="1"
                    max="20"
                    className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                    value={currentCustomer.count}
                    onChange={handleInputChange}
                  />
                </div>
              </div>
              
              <div className="px-6 py-4 border-t bg-gray-50 flex justify-end space-x-2">
                <button
                  onClick={handleAddModalClose}
                  className="px-4 py-2 border border-gray-300 rounded-md text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
                >
                  취소
                </button>
                <button
                  onClick={handleAddCustomer}
                  className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
                >
                  <UserPlus size={16} className="inline-block mr-1" />
                  등록하기
                </button>
              </div>
            </div>
          </div>
        )}

        {/* 고객 정보 수정 모달 */}
        {isEditModalOpen && (
          <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
            <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
              <div className="flex justify-between items-center border-b px-6 py-4">
                <h3 className="text-lg font-medium text-gray-900">
                  고객 정보 수정 (ID: {currentCustomer.id})
                </h3>
                <button
                  onClick={handleEditModalClose}
                  className="text-gray-400 hover:text-gray-500"
                >
                  <X size={20} />
                </button>
              </div>
              
              <div className="px-6 py-4">
                <div className="mb-4">
                  <label className="block text-sm font-medium text-gray-700 mb-1">
                    인원 수
                  </label>
                  <input
                    type="number"
                    name="count"
                    min="1"
                    max="20"
                    className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                    value={currentCustomer.count}
                    onChange={handleInputChange}
                  />
                </div>
              </div>
              
              <div className="px-6 py-4 border-t bg-gray-50 flex justify-end space-x-2">
                <button
                  onClick={handleEditModalClose}
                  className="px-4 py-2 border border-gray-300 rounded-md text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
                >
                  취소
                </button>
                <button
                  onClick={handleEditCustomer}
                  className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-green-600 hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-green-500"
                >
                  <Save size={16} className="inline-block mr-1" />
                  저장하기
                </button>
              </div>
            </div>
          </div>
        )}

        {/* 확인 모달 */}
        {isConfirmModalOpen && (
          <div className="fixed inset-0 bg-gray-600 bg-opacity-50 overflow-y-auto h-full w-full flex items-center justify-center z-50">
            <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
              <div className="flex justify-between items-center border-b px-6 py-4">
                <h3 className="text-lg font-medium text-gray-900">
                  {confirmAction.type === 'assign' ? '테이블 배정 확인' : '테이블 배정 해제 확인'}
                </h3>
                <button
                  onClick={handleCancelAction}
                  className="text-gray-400 hover:text-gray-500"
                >
                  <X size={20} />
                </button>
              </div>
              
              <div className="px-6 py-4">
                {confirmAction.type === 'assign' && (
                  <p className="text-gray-700">
                    <span className="font-medium">테이블 {confirmAction.data?.tableId}번</span>에 
                    <span className="font-medium"> {selectedCustomer?.count}명</span> 고객을 배정하시겠습니까?
                  </p>
                )}
                
                {confirmAction.type === 'release' && (
                  <p className="text-gray-700">
                    <span className="font-medium">테이블 {confirmAction.data?.tableId}번</span>의 배정을 해제하시겠습니까?
                  </p>
                )}
              </div>
              
              <div className="px-6 py-4 border-t bg-gray-50 flex justify-end space-x-2">
                <button
                  onClick={handleCancelAction}
                  className="px-4 py-2 border border-gray-300 rounded-md text-sm font-medium text-gray-700 bg-white hover:bg-gray-50"
                >
                  취소
                </button>
                <button
                  onClick={handleConfirmAction}
                  className={`px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white ${
                    confirmAction.type === 'assign' 
                      ? 'bg-blue-600 hover:bg-blue-700 focus:ring-blue-500' 
                      : 'bg-red-600 hover:bg-red-700 focus:ring-red-500'
                  } focus:outline-none focus:ring-2 focus:ring-offset-2`}
                >
                  {confirmAction.type === 'assign' ? '배정하기' : '배정 해제하기'}
                </button>
              </div>
            </div>
          </div>
        )}
      </Layout>
    );
  };
  
  export default CustomerPage;