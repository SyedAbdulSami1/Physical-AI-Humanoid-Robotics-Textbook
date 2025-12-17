import React, { useState } from 'react';
import { authClient } from '@site/src/utils/authClient';

const SigninModal = ({ isOpen, onClose }) => {
  if (!isOpen) return null;

  const [formData, setFormData] = useState({
    email: '',
    password: ''
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSignin = async () => {
    setLoading(true);
    setError('');

    try {
      const result = await authClient.signIn.email({
        email: formData.email,
        password: formData.password,
      });

      if (result?.error) {
        setError(result.error?.message || 'Signin failed');
        setLoading(false);
        return;
      }

      // Close modal and redirect or show success
      onClose();
      // Optionally redirect to dashboard or show welcome message
    } catch (err) {
      setError('An error occurred during signin');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') {
      handleSignin();
    }
  };

  return (
    <div className="modal-overlay" onClick={onClose}>
      <div className="modal-content" onClick={(e) => e.stopPropagation()}>
        <div className="modal-header">
          <h2>Sign In</h2>
          <button className="close-button" onClick={onClose}>×</button>
        </div>
        <div className="modal-body">
          <div className="form-group">
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              required
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              name="password"
              value={formData.password}
              onChange={handleChange}
              onKeyDown={handleKeyDown}
              required
            />
          </div>
          
          <div className="form-actions">
            <button onClick={handleSignin} disabled={loading}>
              {loading ? 'Signing In...' : 'Sign In'}
            </button>
          </div>
          
          <div className="auth-links">
            <p>Don't have an account? <a href="#" onClick={(e) => { e.preventDefault(); onClose(); window.location.href = '/auth/signup'; }}>Sign up</a></p>
          </div>
          
          {error && <div className="error">{error}</div>}
        </div>
      </div>
    </div>
  );
};

export default SigninModal;