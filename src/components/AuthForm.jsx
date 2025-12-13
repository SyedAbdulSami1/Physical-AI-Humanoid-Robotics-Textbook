import React, { useState } from 'react';

const AuthForm = () => {
  const [isLogin, setIsLogin] = useState(true);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');

  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setError('');

    try {
      const endpoint = isLogin ? '/api/v1/auth/login' : '/api/v1/auth/register';
      const payload = isLogin 
        ? { email, password } 
        : { email, name, password };

      const response = await fetch(endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(payload)
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Authentication failed');
      }

      // Store the token and user info
      localStorage.setItem('access_token', data.access_token);
      localStorage.setItem('user', JSON.stringify(data.user));
      
      // Redirect or update UI as needed
      window.location.reload();
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="auth-form-container">
      <div className="auth-form">
        <h2>{isLogin ? 'Login' : 'Sign Up'}</h2>
        
        {error && <div className="error-message">{error}</div>}
        
        <form onSubmit={handleSubmit}>
          {!isLogin && (
            <div className="form-group">
              <label htmlFor="name">Full Name</label>
              <input
                type="text"
                id="name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                required={!isLogin}
                disabled={isLoading}
              />
            </div>
          )}
          
          <div className="form-group">
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              disabled={isLoading}
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              disabled={isLoading}
            />
          </div>
          
          <button 
            type="submit" 
            disabled={isLoading}
            className="auth-submit-button"
          >
            {isLoading ? 'Processing...' : (isLogin ? 'Login' : 'Sign Up')}
          </button>
        </form>
        
        <div className="auth-switch">
          <p>
            {isLogin 
              ? "Don't have an account? " 
              : "Already have an account? "}
            <button 
              type="button" 
              onClick={() => {
                setIsLogin(!isLogin);
                setError('');
              }}
              disabled={isLoading}
              className="auth-switch-button"
            >
              {isLogin ? 'Sign Up' : 'Login'}
            </button>
          </p>
        </div>
      </div>
    </div>
  );
};

export default AuthForm;