import React from 'react';
import { Outlet } from 'react-router-dom';
import Sidebar from './Sidebar';
import NotificationOverlay from '../Notification/NotificationOverlay';

const Layout = ({ 
  selectedCategory, 
  onSelectCategory, 
  notifications, 
  onCloseNotification 
}) => {
  return (
    <div className="flex h-screen w-screen bg-[#F7F9FA] overflow-hidden">
      <Sidebar 
        selectedCategory={selectedCategory} 
        onSelectCategory={onSelectCategory} 
      />
      
      <main className="flex-1 overflow-auto relative">
        <Outlet />
        <NotificationOverlay
          notifications={notifications}
          onClose={onCloseNotification}
        />
      </main>
    </div>
  );
};

export default Layout; 