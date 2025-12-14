import React, { useState } from 'react';
import { signIn } from 'next-better-auth/client';

const SignupPage = () => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    softwareExperience: '',
    hardwareExperience: ''
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      // First, register the user with Better-Auth
      const result = await signIn('email', {
        email: formData.email,
        password: formData.password,
        name: formData.name,
        callbackURL: '/dashboard' // Redirect after signup
      });

      if (result?.error) {
        setError(result.error);
      } else {
        // After successful signup, we'll send the questionnaire data to our backend
        const profileResponse = await fetch('/api/user/profile', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            softwareExperience: formData.softwareExperience,
            hardwareExperience: formData.hardwareExperience
          }),
        });

        if (!profileResponse.ok) {
          console.error('Failed to save profile data:', profileResponse.statusText);
        }
      }
    } catch (err) {
      setError('An error occurred during signup');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>Sign Up</h2>
        {error && <div className="error">{error}</div>}
        
        <form onSubmit={handleSubmit}>
          <div className="form-group">
            <label htmlFor="name">Full Name</label>
            <input
              type="text"
              id="name"
              name="name"
              value={formData.name}
              onChange={handleChange}
              required
            />
          </div>
          
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
              required
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="softwareExperience">Software Experience</label>
            <select
              id="softwareExperience"
              name="softwareExperience"
              value={formData.softwareExperience}
              onChange={handleChange}
              required
            >
              <option value="">Select your software experience level</option>
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
              <option value="expert">Expert</option>
            </select>
          </div>
          
          <div className="form-group">
            <label htmlFor="hardwareExperience">Hardware Experience</label>
            <select
              id="hardwareExperience"
              name="hardwareExperience"
              value={formData.hardwareExperience}
              onChange={handleChange}
              required
            >
              <option value="">Select your hardware experience level</option>
              <option value="none">None</option>
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
              <option value="expert">Expert</option>
            </select>
          </div>
          
          <button type="submit" disabled={loading}>
            {loading ? 'Signing up...' : 'Sign Up'}
          </button>
        </form>
        
        <div className="auth-link">
          Already have an account? <a href="/auth/login">Log in</a>
        </div>
      </div>
    </div>
  );
};

export default SignupPage;