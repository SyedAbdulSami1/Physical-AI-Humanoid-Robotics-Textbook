import React, { createContext, useContext, useState, useEffect } from 'react';

// Create Auth Context
const AuthContext = createContext();

// Auth Provider Component
export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [isPending, setIsPending] = useState(true);

  useEffect(() => {
    // Check if user data exists in localStorage
    const storedUser = localStorage.getItem('user');
    if (storedUser) {
      setUser(JSON.parse(storedUser));
    }
    setIsPending(false);
  }, []);

  const logout = () => {
    localStorage.removeItem('user');
    setUser(null);
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