import React, { useState, useEffect } from 'react';
import Layout from '../components/Layout';
import RobotStatusPanel from '../components/dashboard/RobotStatusPanel';
import StoreMap from '../components/dashboard/StoreMap';
import EventTimeline from '../components/dashboard/EventTimeline';

const DashboardPage = () => {
  const [robots, setRobots] = useState([]);
  const [tables, setTables] = useState([]);
  const [events, setEvents] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [selectedEvent, setSelectedEvent] = useState(null);

  useEffect(() => {
    const fetchDashboardData = async () => {
      setIsLoading(true);
      setError(null);

      try {
        // Fetch robots data
        const robotsResponse = await fetch('/api/robots');
        if (!robotsResponse.ok) throw new Error('Failed to fetch robots data');
        const robotsData = await robotsResponse.json();
        setRobots(robotsData);

        // Fetch tables data
        const tablesResponse = await fetch('/api/tables');
        if (!tablesResponse.ok) throw new Error('Failed to fetch tables data');
        const tablesData = await tablesResponse.json();
        setTables(tablesData);

        // Fetch events data
        const eventsResponse = await fetch('/api/events');
        if (!eventsResponse.ok) throw new Error('Failed to fetch events data');
        const eventsData = await eventsResponse.json();
        setEvents(eventsData);

      } catch (error) {
        console.error('Error fetching dashboard data:', error);
        setError('데이터를 불러오는 중 오류가 발생했습니다');
      } finally {
        setIsLoading(false);
      }
    };

    fetchDashboardData();

    // Set up polling for real-time updates every 30 seconds
    const intervalId = setInterval(fetchDashboardData, 30000);
    
    return () => clearInterval(intervalId);
  }, []);

  const handleEventSelect = (event) => {
    setSelectedEvent(event);
  };

  return (
    <Layout>
      <h1 className="text-2xl font-bold text-gray-800 mb-6">대시보드</h1>
      
      {error && (
        <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6">
          {error}
        </div>
      )}
      
      {isLoading ? (
        <div className="flex items-center justify-center h-64">
          <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
        </div>
      ) : (
        <section className="grid grid-cols-1 lg:grid-cols-3 gap-4 p-4">
          <RobotStatusPanel robots={robots} />
          <StoreMap tables={tables} robots={robots} selectedEvent={selectedEvent} />
          <EventTimeline events={events} onEventSelect={handleEventSelect} />
        </section>
      )}
    </Layout>
  );
};

export default DashboardPage; 