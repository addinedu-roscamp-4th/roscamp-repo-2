import React, { useState, useEffect } from 'react';
import { 
  Settings as SettingsIcon, AlertTriangle, Save, 
  Bell, Clock, RefreshCw, PieChart, Database
} from 'react-feather';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';

const SettingsPage = () => {
  const [generalSettings, setGeneralSettings] = useState({
    store_name: '',
    operation_start: '09:00',
    operation_end: '22:00',
    auto_robot_assignment: true,
    auto_inventory_alert: true,
    inventory_threshold: 20
  });
  
  const [notificationSettings, setNotificationSettings] = useState({
    email_notifications: true,
    push_notifications: true,
    low_stock_notifications: true,
    order_notifications: true,
    emergency_notifications: true
  });
  
  const [isLoading, setIsLoading] = useState(true);
  const [saveLoading, setSaveLoading] = useState(false);
  const [error, setError] = useState(null);
  const [saveMessage, setSaveMessage] = useState('');
  const { apiCall } = useAuth();

  useEffect(() => {
    const fetchSettings = async () => {
      setIsLoading(true);
      setError(null);
      try {
        const data = await apiCall('/api/settings');
        
        // 설정 데이터가 있을 경우 상태 업데이트
        if (data.general) {
          setGeneralSettings(data.general);
        }
        if (data.notifications) {
          setNotificationSettings(data.notifications);
        }
      } catch (err) {
        console.error('Failed to load settings:', err);
        setError('설정을 불러올 수 없습니다');
      } finally {
        setIsLoading(false);
      }
    };

    fetchSettings();
  }, [apiCall]);

  const handleGeneralChange = (e) => {
    const { name, value, type, checked } = e.target;
    setGeneralSettings({
      ...generalSettings,
      [name]: type === 'checkbox' ? checked : value
    });
  };

  const handleNotificationChange = (e) => {
    const { name, checked } = e.target;
    setNotificationSettings({
      ...notificationSettings,
      [name]: checked
    });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setSaveLoading(true);
    setSaveMessage('');
    setError(null);
    
    try {
      await apiCall('/api/settings', {
        method: 'PUT',
        body: JSON.stringify({
          general: generalSettings,
          notifications: notificationSettings
        })
      });
      
      setSaveMessage('설정이 성공적으로 저장되었습니다');
      
      // 3초 후 저장 메시지 숨기기
      setTimeout(() => {
        setSaveMessage('');
      }, 3000);
    } catch (err) {
      console.error('Failed to save settings:', err);
      setError('설정을 저장하는 중 오류가 발생했습니다');
    } finally {
      setSaveLoading(false);
    }
  };

  return (
    <Layout>
      <div className="container mx-auto p-4">
        <div className="flex justify-between items-center mb-6">
          <h1 className="text-2xl font-bold text-gray-800 flex items-center">
            <SettingsIcon className="text-blue-600 mr-2" size={28} />
            설정
          </h1>
        </div>

        {error && (
          <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6 flex items-center">
            <AlertTriangle className="mr-2" size={20} />
            <span>{error}</span>
          </div>
        )}
        
        {saveMessage && (
          <div className="bg-green-100 border border-green-400 text-green-700 px-4 py-3 rounded mb-6 flex items-center">
            <div className="mr-2">✓</div>
            <span>{saveMessage}</span>
          </div>
        )}

        {isLoading ? (
          <div className="flex items-center justify-center h-64">
            <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
          </div>
        ) : (
          <div className="bg-white rounded-lg shadow overflow-hidden">
            <form onSubmit={handleSubmit}>
              <div className="p-6 border-b">
                <h2 className="text-lg font-medium text-gray-800 mb-4 flex items-center">
                  <Clock size={20} className="mr-2 text-blue-600" />
                  일반 설정
                </h2>
                
                <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                  <div>
                    <label htmlFor="store_name" className="block text-sm font-medium text-gray-700 mb-1">
                      매장 이름
                    </label>
                    <input
                      type="text"
                      name="store_name"
                      id="store_name"
                      value={generalSettings.store_name}
                      onChange={handleGeneralChange}
                      className="shadow-sm focus:ring-blue-500 focus:border-blue-500 block w-full sm:text-sm border-gray-300 rounded-md"
                    />
                  </div>
                  
                  <div className="grid grid-cols-2 gap-4">
                    <div>
                      <label htmlFor="operation_start" className="block text-sm font-medium text-gray-700 mb-1">
                        운영 시작 시간
                      </label>
                      <input
                        type="time"
                        name="operation_start"
                        id="operation_start"
                        value={generalSettings.operation_start}
                        onChange={handleGeneralChange}
                        className="shadow-sm focus:ring-blue-500 focus:border-blue-500 block w-full sm:text-sm border-gray-300 rounded-md"
                      />
                    </div>
                    
                    <div>
                      <label htmlFor="operation_end" className="block text-sm font-medium text-gray-700 mb-1">
                        운영 종료 시간
                      </label>
                      <input
                        type="time"
                        name="operation_end"
                        id="operation_end"
                        value={generalSettings.operation_end}
                        onChange={handleGeneralChange}
                        className="shadow-sm focus:ring-blue-500 focus:border-blue-500 block w-full sm:text-sm border-gray-300 rounded-md"
                      />
                    </div>
                  </div>
                  
                  <div>
                    <label htmlFor="inventory_threshold" className="block text-sm font-medium text-gray-700 mb-1">
                      재고 알림 기준값 (%)
                    </label>
                    <input
                      type="number"
                      name="inventory_threshold"
                      id="inventory_threshold"
                      min="1"
                      max="100"
                      value={generalSettings.inventory_threshold}
                      onChange={handleGeneralChange}
                      className="shadow-sm focus:ring-blue-500 focus:border-blue-500 block w-full sm:text-sm border-gray-300 rounded-md"
                    />
                    <p className="mt-1 text-sm text-gray-500">재고가 이 기준값보다 낮으면 알림이 발생합니다.</p>
                  </div>
                  
                  <div className="space-y-4">
                    <div className="flex items-start">
                      <div className="flex items-center h-5">
                        <input
                          id="auto_robot_assignment"
                          name="auto_robot_assignment"
                          type="checkbox"
                          checked={generalSettings.auto_robot_assignment}
                          onChange={handleGeneralChange}
                          className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                        />
                      </div>
                      <div className="ml-3 text-sm">
                        <label htmlFor="auto_robot_assignment" className="font-medium text-gray-700">
                          로봇 자동 배정
                        </label>
                        <p className="text-gray-500">주문 접수 시 로봇을 자동으로 배정합니다.</p>
                      </div>
                    </div>
                    
                    <div className="flex items-start">
                      <div className="flex items-center h-5">
                        <input
                          id="auto_inventory_alert"
                          name="auto_inventory_alert"
                          type="checkbox"
                          checked={generalSettings.auto_inventory_alert}
                          onChange={handleGeneralChange}
                          className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                        />
                      </div>
                      <div className="ml-3 text-sm">
                        <label htmlFor="auto_inventory_alert" className="font-medium text-gray-700">
                          재고 자동 알림
                        </label>
                        <p className="text-gray-500">재고가 부족할 때 자동으로 알림을 받습니다.</p>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
              
              <div className="p-6 border-b">
                <h2 className="text-lg font-medium text-gray-800 mb-4 flex items-center">
                  <Bell size={20} className="mr-2 text-blue-600" />
                  알림 설정
                </h2>
                
                <div className="space-y-4">
                  <div className="flex items-start">
                    <div className="flex items-center h-5">
                      <input
                        id="email_notifications"
                        name="email_notifications"
                        type="checkbox"
                        checked={notificationSettings.email_notifications}
                        onChange={handleNotificationChange}
                        className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                      />
                    </div>
                    <div className="ml-3 text-sm">
                      <label htmlFor="email_notifications" className="font-medium text-gray-700">
                        이메일 알림
                      </label>
                      <p className="text-gray-500">중요 알림을 이메일로 받습니다.</p>
                    </div>
                  </div>
                  
                  <div className="flex items-start">
                    <div className="flex items-center h-5">
                      <input
                        id="push_notifications"
                        name="push_notifications"
                        type="checkbox"
                        checked={notificationSettings.push_notifications}
                        onChange={handleNotificationChange}
                        className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                      />
                    </div>
                    <div className="ml-3 text-sm">
                      <label htmlFor="push_notifications" className="font-medium text-gray-700">
                        푸시 알림
                      </label>
                      <p className="text-gray-500">브라우저 푸시 알림을 받습니다.</p>
                    </div>
                  </div>
                  
                  <div className="mt-6 space-y-2">
                    <h3 className="text-sm font-medium text-gray-700">알림 유형</h3>
                    <div className="ml-6 space-y-4">
                      <div className="flex items-start">
                        <div className="flex items-center h-5">
                          <input
                            id="low_stock_notifications"
                            name="low_stock_notifications"
                            type="checkbox"
                            checked={notificationSettings.low_stock_notifications}
                            onChange={handleNotificationChange}
                            className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                          />
                        </div>
                        <div className="ml-3 text-sm">
                          <label htmlFor="low_stock_notifications" className="font-medium text-gray-700">
                            재고 부족 알림
                          </label>
                        </div>
                      </div>
                      
                      <div className="flex items-start">
                        <div className="flex items-center h-5">
                          <input
                            id="order_notifications"
                            name="order_notifications"
                            type="checkbox"
                            checked={notificationSettings.order_notifications}
                            onChange={handleNotificationChange}
                            className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                          />
                        </div>
                        <div className="ml-3 text-sm">
                          <label htmlFor="order_notifications" className="font-medium text-gray-700">
                            주문 알림
                          </label>
                        </div>
                      </div>
                      
                      <div className="flex items-start">
                        <div className="flex items-center h-5">
                          <input
                            id="emergency_notifications"
                            name="emergency_notifications"
                            type="checkbox"
                            checked={notificationSettings.emergency_notifications}
                            onChange={handleNotificationChange}
                            className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                          />
                        </div>
                        <div className="ml-3 text-sm">
                          <label htmlFor="emergency_notifications" className="font-medium text-gray-700">
                            비상 상황 알림
                          </label>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
              
              <div className="px-6 py-4 bg-gray-50 flex justify-end">
                <button
                  type="submit"
                  disabled={saveLoading}
                  className={`inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md shadow-sm ${
                    saveLoading 
                      ? 'bg-blue-400 text-white cursor-not-allowed' 
                      : 'bg-blue-600 text-white hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500'
                  }`}
                >
                  {saveLoading ? (
                    <>
                      <RefreshCw size={16} className="mr-2 animate-spin" />
                      저장 중...
                    </>
                  ) : (
                    <>
                      <Save size={16} className="mr-2" />
                      설정 저장
                    </>
                  )}
                </button>
              </div>
            </form>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default SettingsPage; 