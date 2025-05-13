import React, { useState, useEffect } from 'react';
import Layout from '../components/Layout';
import { AlertCircle, Calendar, TrendingUp, RefreshCw, ThumbsUp } from 'react-feather';
import { BarChart, Bar, LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';

const StatsPage = () => {
  const [orderStats, setOrderStats] = useState([]);
  const [inventoryStats, setInventoryStats] = useState([]);
  const [recommendedItems, setRecommendedItems] = useState([]);
  const [periodType, setPeriodType] = useState('day'); // day, week, month
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    fetchData();
  }, [periodType]);

  const fetchData = async () => {
    setIsLoading(true);
    setError(null);

    try {
      // Fetch order statistics
      const orderResponse = await fetch(`/api/orders/stats?period=${periodType}`);
      if (!orderResponse.ok) throw new Error('Failed to fetch order statistics');
      const orderData = await orderResponse.json();
      setOrderStats(orderData);

      // Fetch inventory statistics
      const inventoryResponse = await fetch('/api/inventories/stats');
      if (!inventoryResponse.ok) throw new Error('Failed to fetch inventory statistics');
      const inventoryData = await inventoryResponse.json();
      setInventoryStats(inventoryData);

      // Fetch recommended menu items
      const recommendedResponse = await fetch('/api/menu/recommended');
      if (!recommendedResponse.ok) throw new Error('Failed to fetch recommended menu items');
      const recommendedData = await recommendedResponse.json();
      setRecommendedItems(recommendedData);
    } catch (error) {
      console.error('Error fetching statistics:', error);
      setError('통계 데이터를 불러오는 중 오류가 발생했습니다');
    } finally {
      setIsLoading(false);
    }
  };

  const formatCurrency = (value) => {
    return new Intl.NumberFormat('ko-KR', {
      style: 'currency',
      currency: 'KRW',
      minimumFractionDigits: 0,
    }).format(value);
  };

  // Simulate inventory gauges data
  const getInventoryData = () => {
    if (!inventoryStats || inventoryStats.length === 0) {
      return [];
    }
    
    return inventoryStats.map(item => ({
      name: item.name,
      value: item.stock_level,
      total: item.max_capacity,
      status: item.status,
    })).slice(0, 6); // Show top 6 items
  };

  const determineGaugeColor = (status) => {
    switch (status) {
      case 'OUT_OF_STOCK':
        return '#EF4444'; // red
      case 'LOW_STOCK':
        return '#F59E0B'; // amber
      case 'IN_STOCK':
        return '#10B981'; // green
      default:
        return '#9CA3AF'; // gray
    }
  };

  return (
    <Layout>
      <div className="flex justify-between items-center mb-6">
        <h1 className="text-2xl font-bold text-gray-800">주문·재고 통계</h1>
        <button
          onClick={fetchData}
          className="flex items-center text-blue-600 hover:text-blue-800"
        >
          <RefreshCw size={16} className="mr-1" />
          새로고침
        </button>
      </div>

      {error && (
        <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6 flex items-center">
          <AlertCircle size={20} className="mr-2" />
          {error}
        </div>
      )}

      {isLoading ? (
        <div className="flex items-center justify-center h-64">
          <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
        </div>
      ) : (
        <>
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
            {/* Order Charts */}
            <div className="bg-white rounded-lg shadow p-4">
              <div className="flex justify-between items-center mb-4">
                <h2 className="text-lg font-semibold text-gray-800 flex items-center">
                  <TrendingUp size={20} className="mr-2 text-blue-500" />
                  주문 통계
                </h2>
                <div className="flex space-x-2">
                  <button
                    onClick={() => setPeriodType('day')}
                    className={`px-3 py-1 text-xs rounded-full ${
                      periodType === 'day'
                        ? 'bg-blue-600 text-white'
                        : 'bg-gray-200 text-gray-700'
                    }`}
                  >
                    일별
                  </button>
                  <button
                    onClick={() => setPeriodType('week')}
                    className={`px-3 py-1 text-xs rounded-full ${
                      periodType === 'week'
                        ? 'bg-blue-600 text-white'
                        : 'bg-gray-200 text-gray-700'
                    }`}
                  >
                    주별
                  </button>
                  <button
                    onClick={() => setPeriodType('month')}
                    className={`px-3 py-1 text-xs rounded-full ${
                      periodType === 'month'
                        ? 'bg-blue-600 text-white'
                        : 'bg-gray-200 text-gray-700'
                    }`}
                  >
                    월별
                  </button>
                </div>
              </div>

              <div className="h-72">
                {orderStats.length > 0 ? (
                  <ResponsiveContainer width="100%" height="100%">
                    <BarChart
                      data={orderStats}
                      margin={{ top: 5, right: 30, left: 20, bottom: 5 }}
                    >
                      <CartesianGrid strokeDasharray="3 3" />
                      <XAxis dataKey="period" />
                      <YAxis />
                      <Tooltip 
                        formatter={(value) => [formatCurrency(value), '매출']}
                      />
                      <Legend />
                      <Bar name="매출액" dataKey="amount" fill="#3B82F6" />
                    </BarChart>
                  </ResponsiveContainer>
                ) : (
                  <div className="flex items-center justify-center h-full text-gray-500">
                    주문 데이터가 없습니다
                  </div>
                )}
              </div>

              <div className="mt-6 h-72">
                {orderStats.length > 0 ? (
                  <ResponsiveContainer width="100%" height="100%">
                    <LineChart
                      data={orderStats}
                      margin={{ top: 5, right: 30, left: 20, bottom: 5 }}
                    >
                      <CartesianGrid strokeDasharray="3 3" />
                      <XAxis dataKey="period" />
                      <YAxis />
                      <Tooltip />
                      <Legend />
                      <Line 
                        name="주문 건수" 
                        type="monotone" 
                        dataKey="orders" 
                        stroke="#10B981" 
                        activeDot={{ r: 8 }} 
                      />
                    </LineChart>
                  </ResponsiveContainer>
                ) : (
                  <div className="flex items-center justify-center h-full text-gray-500">
                    주문 데이터가 없습니다
                  </div>
                )}
              </div>
            </div>

            {/* Inventory Gauges */}
            <div className="bg-white rounded-lg shadow p-4">
              <h2 className="text-lg font-semibold text-gray-800 mb-4">재고 현황</h2>
              
              <div className="grid grid-cols-2 gap-4">
                {getInventoryData().map((item, index) => (
                  <div key={index} className="bg-gray-50 p-3 rounded-lg">
                    <div className="flex justify-between items-center mb-2">
                      <div className="font-medium">{item.name}</div>
                      <div className={`px-2 py-1 text-xs rounded-full ${
                        item.status === 'OUT_OF_STOCK'
                          ? 'bg-red-100 text-red-800'
                          : item.status === 'LOW_STOCK'
                          ? 'bg-yellow-100 text-yellow-800'
                          : 'bg-green-100 text-green-800'
                      }`}>
                        {item.status.replace('_', ' ')}
                      </div>
                    </div>
                    
                    <div className="mb-1 text-sm flex justify-between">
                      <span>재고: {item.value}</span>
                      <span>총 용량: {item.total}</span>
                    </div>
                    
                    <div className="w-full bg-gray-200 rounded-full h-4">
                      <div 
                        className="h-4 rounded-full" 
                        style={{
                          width: `${Math.min(100, (item.value / item.total) * 100)}%`,
                          backgroundColor: determineGaugeColor(item.status)
                        }}
                      ></div>
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </div>

          {/* Recommended Menu Items */}
          <div className="bg-white rounded-lg shadow p-4">
            <h2 className="text-lg font-semibold text-gray-800 mb-4 flex items-center">
              <ThumbsUp size={20} className="mr-2 text-green-500" />
              추천 메뉴
            </h2>
            
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
              {recommendedItems.length > 0 ? (
                recommendedItems.map((item) => (
                  <div key={item.id} className="flex border rounded-lg overflow-hidden">
                    <div 
                      className="w-24 h-24 bg-gray-200 flex-shrink-0"
                      style={{
                        backgroundImage: `url(${item.image_url || '/static/placeholder.jpg'})`,
                        backgroundSize: 'cover',
                        backgroundPosition: 'center'
                      }}
                    ></div>
                    <div className="p-3 flex-1">
                      <div className="font-medium">{item.name}</div>
                      <div className="text-sm text-gray-600 mb-1">{formatCurrency(item.price)}</div>
                      <div className="text-xs text-gray-500">인기도: {item.popularity_score}</div>
                    </div>
                  </div>
                ))
              ) : (
                <div className="col-span-3 text-center text-gray-500 py-8">
                  추천 메뉴 데이터가 없습니다
                </div>
              )}
            </div>
          </div>
        </>
      )}
    </Layout>
  );
};

export default StatsPage; 