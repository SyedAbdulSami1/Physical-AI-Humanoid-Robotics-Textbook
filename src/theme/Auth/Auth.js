import React, { createContext, useContext, useState, useEffect } from 'react';
import { useUser, signOut } from 'next-better-auth/client/react';

// Create Auth Context
const AuthContext = createContext();

// Auth Provider Component
export const AuthProvider = ({ children }) => {
  const { data: user, isPending } = useUser();

  const logout = async () => {
    await signOut();
  };

  return (
    <AuthContext.Provider value={{ user, isPending, logout }}>
      {children}
    </AuthContext.Provider>
  );
};

// Custom hook to use Auth Context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

// Auth Component for Docusaurus
export const AuthComponent = () => {
  const { user, isPending, logout } = useAuth();

  if (isPending) {
    return <div>Loading...</div>;
  }

  return (
    <div className="auth-component">
      {user ? (
        <div className="user-info">
          <span>Welcome, {user.name || user.email}!</span>
          <button onClick={logout} className="logout-btn">
            Logout
          </button>
        </div>
      ) : (
        <div className="login-prompt">
          <a href="/auth/login">Login</a> or <a href="/auth/signup">Sign Up</a>
        </div>
      )}
    </div>
  );
};

export default AuthComponent;