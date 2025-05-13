import React from 'react';
import Header from './Header';
import Sidebar from './Sidebar';
import Footer from './Footer';

const Layout = ({ children, category, setCategory }) => {
  return (
    <div className="flex flex-col h-screen bg-gray-100">
      <Header />
      
      <div className="flex flex-grow">
        <Sidebar 
          selectedCategory={category} 
          onSelectCategory={setCategory} 
        />
        <main className="flex-grow p-4 overflow-y-auto">
          {children}
        </main>
      </div>
      
      <Footer />
    </div>
  );
};

export default Layout; 