import React, { useState, useEffect } from 'react';
import { authClient } from '@site/src/utils/authClient';
import { useHistory } from '@docusaurus/router';

const SignupPage = () => {
  const [step, setStep] = useState(1);
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    confirmPassword: '',
    softwareExperience: '',
    hardwareExperience: '',
    gpuType: '',
    ramSize: '',
    osType: ''
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const history = useHistory();

  useEffect(() => {
    // Check if already logged in
    const checkSession = async () => {
      try {
        const session = await authClient.getSession();
        if (session?.user) {
          // Redirect to dashboard if already logged in
          history.push('/dashboard');
        }
      } catch (error) {
        console.error('Error checking session:', error);
      }
    };

    checkSession();
  }, [history]);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSignup = async () => {
    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    setLoading(true);
    setError('');

    try {
      // First, sign up the user with Better-Auth
      const result = await authClient.signUp.email({
        email: formData.email,
        password: formData.password,
        name: formData.name,
      });

      if (result?.error) {
        setError(result.error?.message || 'Signup failed');
        setLoading(false);
        return;
      }

      // After successful signup, save the questionnaire data
      const questionnaireResponse = await fetch('/api/user/profile', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${result.session.token}` // Use the session token received from signup
        },
        body: JSON.stringify({
          softwareExperience: formData.softwareExperience,
          hardwareExperience: formData.hardwareExperience,
          gpuType: formData.gpuType,
          ramSize: formData.ramSize,
          osType: formData.osType
        }),
      });

      if (!questionnaireResponse.ok) {
        console.error('Failed to save questionnaire data:', questionnaireResponse.statusText);
      }

      // Redirect to dashboard or show success
      window.location.href = '/dashboard'; // or wherever you want to redirect
    } catch (err) {
      setError('An error occurred during signup');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  const nextStep = () => {
    if (step < 2) setStep(step + 1);
  };

  const prevStep = () => {
    if (step > 1) setStep(step - 1);
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && step === 1) {
      e.preventDefault();
      nextStep();
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-form">
        {step === 1 && (
          <div className="signup-step">
            <h2>Account Information</h2>
            {error && <div className="error">{error}</div>}
            
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
              <label htmlFor="confirmPassword">Confirm Password</label>
              <input
                type="password"
                id="confirmPassword"
                name="confirmPassword"
                value={formData.confirmPassword}
                onChange={handleChange}
                onKeyDown={handleKeyDown}
                required
              />
            </div>
            
            <div className="form-actions">
              <button type="button" onClick={nextStep} disabled={loading}>
                Next
              </button>
            </div>
          </div>
        )}
        
        {step === 2 && (
          <div className="signup-step">
            <h2>Experience Questionnaire</h2>
            {error && <div className="error">{error}</div>}
            
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
            
            <div className="form-group">
              <label htmlFor="gpuType">GPU Type</label>
              <select
                id="gpuType"
                name="gpuType"
                value={formData.gpuType}
                onChange={handleChange}
              >
                <option value="">Select your GPU type</option>
                <option value="none">No dedicated GPU</option>
                <option value="gtx">NVIDIA GTX Series</option>
                <option value="rtx-low">NVIDIA RTX 20xx Series</option>
                <option value="rtx-high">NVIDIA RTX 30xx/40xx Series</option>
                <option value="other">Other</option>
              </select>
            </div>
            
            <div className="form-group">
              <label htmlFor="ramSize">RAM Size</label>
              <select
                id="ramSize"
                name="ramSize"
                value={formData.ramSize}
                onChange={handleChange}
              >
                <option value="">Select your RAM size</option>
                <option value="4gb">4 GB</option>
                <option value="8gb">8 GB</option>
                <option value="16gb">16 GB</option>
                <option value="32gb">32 GB</option>
                <option value="64gb+">64+ GB</option>
              </select>
            </div>
            
            <div className="form-group">
              <label htmlFor="osType">Operating System</label>
              <select
                id="osType"
                name="osType"
                value={formData.osType}
                onChange={handleChange}
              >
                <option value="">Select your OS</option>
                <option value="windows">Windows</option>
                <option value="macos">macOS</option>
                <option value="linux">Linux</option>
              </select>
            </div>
            
            <div className="form-actions">
              <button type="button" onClick={prevStep} disabled={loading}>Previous</button>
              <button type="button" onClick={handleSignup} disabled={loading}>
                {loading ? 'Creating Account...' : 'Sign Up'}
              </button>
            </div>
          </div>
        )}
        
        <div className="auth-link">
          Already have an account? <a href="/auth/login">Sign in</a>
        </div>
      </div>
    </div>
  );
};

export default SignupPage;