import React from 'react';

const AuthLayout = ({ children }) => {
  return (
    <div className="min-h-screen flex items-center justify-center bg-gray-100 py-12 px-4 sm:px-6 lg:px-8">
      <div className="max-w-md w-full space-y-8 bg-white shadow-2xl rounded-2xl">
        <div className="flex justify-center pt-8">
          <h1 className="text-3xl font-bold text-gray-800">RoboDine Admin</h1>
        </div>
        {children}
      </div>
    </div>
  );
};

export default AuthLayout; 