import React, { useState, useEffect } from 'react';
import { 
  Settings as SettingsIcon, AlertTriangle, Save, 
  Bell, Clock, RefreshCw, Server, Database, AlertOctagon
} from 'react-feather';
import Layout from '../components/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useNotifications } from '../contexts/NotificationsContext';

const SettingsPage = () => {
  const [generalSettings, setGeneralSettings] = useState({
    store_name: '',
    operation_start: '09:00',
    operation_end: '22:00',
    inventory_threshold: 20
  });
  
  const [notificationSettings, setNotificationSettings] = useState({
    push_notifications: true,
    INFO: true,
    WARNING: true,
    ERROR: true,
    DEBUG: false
  });
  
  const [isLoading, setIsLoading] = useState(true);
  const [saveLoading, setSaveLoading] = useState(false);
  const [error, setError] = useState(null);
  const [saveMessage, setSaveMessage] = useState('');
  const { apiCall } = useAuth();
  const notificationsContext = useNotifications();
  
  // 방어적 프로그래밍: notificationsContext가 없는 경우를 처리
  const updateSettings = notificationsContext?.updateSettings || (() => {});

  useEffect(() => {
    const fetchSettings = async () => {
      setIsLoading(true);
      setError(null);
      try {
        const data = await apiCall('/api/settings');
        
        // 설정 데이터가 있을 경우 상태 업데이트
        if (data) {
          // 일반 설정 업데이트
          setGeneralSettings({
            store_name: data.store_name || '',
            operation_start: data.operation_start || '09:00',
            operation_end: data.operation_end || '22:00',
            inventory_threshold: data.inventory_threshold || 20
          });
          
          // 알림 설정 업데이트 - LogLevel enum 기반으로 매핑
          if (data.alert_settings) {
            setNotificationSettings({
              push_notifications: data.alert_settings.push_notifications ?? true,
              INFO: data.alert_settings.INFO ?? true,
              WARNING: data.alert_settings.WARNING ?? true,
              ERROR: data.alert_settings.ERROR ?? true,
              DEBUG: data.alert_settings.DEBUG ?? false
            });
          }
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
      // API 요청 데이터 준비
      const requestData = {
        store_name: generalSettings.store_name,
        operation_start: generalSettings.operation_start,
        operation_end: generalSettings.operation_end,
        inventory_threshold: parseInt(generalSettings.inventory_threshold),
        alert_settings: {
          push_notifications: notificationSettings.push_notifications,
          INFO: notificationSettings.INFO,
          WARNING: notificationSettings.WARNING,
          ERROR: notificationSettings.ERROR,
          DEBUG: notificationSettings.DEBUG
        }
      };
      console.log('Saving settings:', requestData);
      
      await apiCall('/api/settings', 'PUT', requestData);
      
      // 알림 설정 업데이트
      updateSettings({
        INFO: notificationSettings.INFO,
        WARNING: notificationSettings.WARNING,
        ERROR: notificationSettings.ERROR,
        DEBUG: notificationSettings.DEBUG
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
                            id="INFO"
                            name="INFO"
                            type="checkbox"
                            checked={notificationSettings.INFO}
                            onChange={handleNotificationChange}
                            className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                          />
                        </div>
                        <div className="ml-3 text-sm flex items-center">
                          <Server className="text-blue-500 mr-2" size={16} />
                          <label htmlFor="INFO" className="font-medium text-gray-700">
                            정보 (INFO) 알림
                          </label>
                        </div>
                      </div>
                      
                      <div className="flex items-start">
                        <div className="flex items-center h-5">
                          <input
                            id="WARNING"
                            name="WARNING"
                            type="checkbox"
                            checked={notificationSettings.WARNING}
                            onChange={handleNotificationChange}
                            className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                          />
                        </div>
                        <div className="ml-3 text-sm flex items-center">
                          <AlertTriangle className="text-yellow-500 mr-2" size={16} />
                          <label htmlFor="WARNING" className="font-medium text-gray-700">
                            경고 (WARNING) 알림
                          </label>
                        </div>
                      </div>
                      
                      <div className="flex items-start">
                        <div className="flex items-center h-5">
                          <input
                            id="ERROR"
                            name="ERROR"
                            type="checkbox"
                            checked={notificationSettings.ERROR}
                            onChange={handleNotificationChange}
                            className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                          />
                        </div>
                        <div className="ml-3 text-sm flex items-center">
                          <AlertOctagon className="text-red-500 mr-2" size={16} />
                          <label htmlFor="ERROR" className="font-medium text-gray-700">
                            오류 (ERROR) 알림
                          </label>
                        </div>
                      </div>

                      <div className="flex items-start">
                        <div className="flex items-center h-5">
                          <input
                            id="DEBUG"
                            name="DEBUG"
                            type="checkbox"
                            checked={notificationSettings.DEBUG}
                            onChange={handleNotificationChange}
                            className="focus:ring-blue-500 h-4 w-4 text-blue-600 border-gray-300 rounded"
                          />
                        </div>
                        <div className="ml-3 text-sm flex items-center">
                          <Database className="text-green-500 mr-2" size={16} />
                          <label htmlFor="DEBUG" className="font-medium text-gray-700">
                            디버그 (DEBUG) 알림
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