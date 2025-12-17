import React, { createContext, useContext, useState, useEffect } from 'react';
import { authClient } from '@site/src/utils/authClient';

// Create Auth Context
const AuthContext = createContext();

// Auth Provider Component
export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [isPending, setIsPending] = useState(true);

  useEffect(() => {
    // Initialize auth state using Better-Auth
    const initAuth = async () => {
      try {
        const session = await authClient.getSession();
        if (session?.user) {
          setUser(session.user);
        }
      } catch (error) {
        console.error('Error initializing auth:', error);
      } finally {
        setIsPending(false);
      }
    };

    initAuth();
  }, []);

  const logout = async () => {
    try {
      await authClient.signOut();
      setUser(null);
    } catch (error) {
      console.error('Error during logout:', error);
      // Fallback to clearing user state if API call fails
      setUser(null);
    }
  };

  const value = {
    user,
    isPending,
    logout,
    signIn: authClient.signIn,
    signUp: authClient.signUp,
    getSession: authClient.getSession
  };

  return (
    <AuthContext.Provider value={value}>
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
          <a href="#" onClick={(e) => { e.preventDefault(); window.location.href = '/auth/login'; }}>Login</a> or <a href="#" onClick={(e) => { e.preventDefault(); window.location.href = '/auth/signup'; }}>Sign Up</a>
        </div>
      )}
    </div>
  );
};

export default AuthComponent;