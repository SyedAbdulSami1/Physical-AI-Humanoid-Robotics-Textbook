import React, { useState } from 'react';
import { authClient } from '@site/src/utils/authClient';

const SignupModal = ({ isOpen, onClose }) => {
  if (!isOpen) return null;

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

      // Close modal and redirect or show success
      onClose();
      // Optionally redirect to dashboard or show success message
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

  return (
    <div className="modal-overlay" onClick={onClose}>
      <div className="modal-content" onClick={(e) => e.stopPropagation()}>
        <div className="modal-header">
          <h2>Sign Up</h2>
          <button className="close-button" onClick={onClose}>×</button>
        </div>
        <div className="modal-body">
          {step === 1 && (
            <div className="signup-step">
              <h3>Account Information</h3>
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
                  required
                />
              </div>
              
              <div className="step-navigation">
                <button onClick={nextStep} disabled={loading}>Next</button>
              </div>
            </div>
          )}
          
          {step === 2 && (
            <div className="signup-step">
              <h3>Experience Questionnaire</h3>
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
              
              <div className="step-navigation">
                <button onClick={prevStep} disabled={loading}>Previous</button>
                <button onClick={handleSignup} disabled={loading}>
                  {loading ? 'Creating Account...' : 'Sign Up'}
                </button>
              </div>
            </div>
          )}
          
          {error && <div className="error">{error}</div>}
        </div>
      </div>
    </div>
  );
};

export default SignupModal;