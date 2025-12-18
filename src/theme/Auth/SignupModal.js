import React, { useState } from 'react';
import { authClient } from '@site/src/utils/authClient';

// Password strength checker function
const checkPasswordStrength = (password) => {
  let score = 0;
  let feedback = [];

  if (password.length >= 8) score++;
  else feedback.push("At least 8 characters");

  if (/[A-Z]/.test(password)) score++;
  else feedback.push("1 uppercase letter");

  if (/[0-9]/.test(password)) score++;
  else feedback.push("1 number");

  if (/[^A-Za-z0-9]/.test(password)) score++;
  else feedback.push("1 symbol");

  const strengthLabels = ["Very Weak", "Weak", "Fair", "Good", "Strong"];
  const strengthColors = ["#ff6b6b", "#ff9e6d", "#ffd166", "#a0e6a0", "#4ade80"];

  return {
    score,
    strength: strengthLabels[score],
    color: strengthColors[score],
    feedback: feedback,
    isValid: score === 4
  };
};

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
  const [passwordStrength, setPasswordStrength] = useState(null);
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));

    // Check password strength when password field changes
    if (name === 'password') {
      setPasswordStrength(checkPasswordStrength(value));
    }
  };

  const handleSignup = async () => {
    // Validate password match
    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    // Validate password requirements
    if (passwordStrength && !passwordStrength.isValid) {
      setError('Please meet all password requirements');
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
    if (step < 2) {
      // Validate step 1 before proceeding
      if (!formData.name || !formData.email || !formData.password || !formData.confirmPassword) {
        setError('Please fill in all required fields');
        return;
      }

      if (formData.password !== formData.confirmPassword) {
        setError('Passwords do not match');
        return;
      }

      if (passwordStrength && !passwordStrength.isValid) {
        setError('Please meet all password requirements');
        return;
      }

      setError('');
      setStep(step + 1);
    }
  };

  const prevStep = () => {
    if (step > 1) setStep(step - 1);
  };

  return (
    <div className="modal-overlay" onClick={onClose}>
      <div className="modal-content" onClick={(e) => e.stopPropagation()}>
        <div className="modal-header">
          <h2>Sign Up for Physical AI & Humanoid Robotics</h2>
          <button className="close-button" onClick={onClose}>×</button>
        </div>
        <div className="modal-body">
          <div className="progress-bar">
            <div className={`step ${step >= 1 ? 'active' : ''}`}>Account</div>
            <div className={`step ${step >= 2 ? 'active' : ''}`}>Profile</div>
          </div>

          {step === 1 && (
            <div className="signup-step">
              <h3>Create Your Account</h3>

              <div className="form-group">
                <label htmlFor="name">
                  Full Name <span className="required">*</span>
                </label>
                <input
                  type="text"
                  id="name"
                  name="name"
                  value={formData.name}
                  onChange={handleChange}
                  placeholder="Enter your full name"
                  required
                />
                <small className="field-help">Your display name in the course</small>
              </div>

              <div className="form-group">
                <label htmlFor="email">
                  Email Address <span className="required">*</span>
                </label>
                <input
                  type="email"
                  id="email"
                  name="email"
                  value={formData.email}
                  onChange={handleChange}
                  placeholder="your.email@example.com"
                  required
                />
                <small className="field-help">We'll use this for course communications</small>
              </div>

              <div className="form-group">
                <label htmlFor="password">
                  Password <span className="required">*</span>
                </label>
                <div className="password-container">
                  <input
                    type={showPassword ? "text" : "password"}
                    id="password"
                    name="password"
                    value={formData.password}
                    onChange={handleChange}
                    placeholder="Create a strong password"
                    required
                  />
                  <button
                    type="button"
                    className="show-password-btn"
                    onClick={() => setShowPassword(!showPassword)}
                  >
                    {showPassword ? '-hide' : 'show'}
                  </button>
                </div>

                <div className="password-strength">
                  <div className="strength-meter">
                    {passwordStrength && (
                      <div
                        className="strength-bar"
                        style={{
                          width: `${(passwordStrength.score / 4) * 100}%`,
                          backgroundColor: passwordStrength.color
                        }}
                      />
                    )}
                  </div>
                  <div className="strength-text">
                    {passwordStrength ? (
                      <span style={{ color: passwordStrength.color }}>
                        {passwordStrength.strength}
                      </span>
                    ) : (
                      <span>Enter a password to check strength</span>
                    )}
                  </div>

                  <div className="password-requirements">
                    <p>Password must contain:</p>
                    <ul>
                      <li className={formData.password.length >= 8 ? 'valid' : 'invalid'}>
                        At least 8 characters
                      </li>
                      <li className={/[A-Z]/.test(formData.password) ? 'valid' : 'invalid'}>
                        1 uppercase letter
                      </li>
                      <li className={/[0-9]/.test(formData.password) ? 'valid' : 'invalid'}>
                        1 number
                      </li>
                      <li className={/[^A-Za-z0-9]/.test(formData.password) ? 'valid' : 'invalid'}>
                        1 symbol
                      </li>
                    </ul>
                  </div>
                </div>
              </div>

              <div className="form-group">
                <label htmlFor="confirmPassword">
                  Confirm Password <span className="required">*</span>
                </label>
                <div className="password-container">
                  <input
                    type={showConfirmPassword ? "text" : "password"}
                    id="confirmPassword"
                    name="confirmPassword"
                    value={formData.confirmPassword}
                    onChange={handleChange}
                    placeholder="Re-enter your password"
                    required
                  />
                  <button
                    type="button"
                    className="show-password-btn"
                    onClick={() => setShowConfirmPassword(!showConfirmPassword)}
                  >
                    {showConfirmPassword ? 'hide' : 'show'}
                  </button>
                </div>
              </div>

              <div className="step-navigation">
                <button
                  onClick={nextStep}
                  disabled={loading || !formData.name || !formData.email || !passwordStrength?.isValid}
                  className="next-step-btn"
                >
                  {loading ? 'Creating Account...' : 'Continue →'}
                </button>
              </div>
            </div>
          )}

          {step === 2 && (
            <div className="signup-step">
              <h3>Tell Us About Your Background</h3>
              <p className="step-description">This helps us personalize your learning experience</p>

              <div className="form-group">
                <label htmlFor="softwareExperience">
                  Software Experience <span className="required">*</span>
                </label>
                <select
                  id="softwareExperience"
                  name="softwareExperience"
                  value={formData.softwareExperience}
                  onChange={handleChange}
                  required
                >
                  <option value="">Select your software experience level</option>
                  <option value="beginner">Beginner - New to programming</option>
                  <option value="intermediate">Intermediate - Some experience</option>
                  <option value="advanced">Advanced - Experienced developer</option>
                  <option value="expert">Expert - Professional/senior developer</option>
                </select>
                <small className="field-help">Your programming background will help personalize content difficulty</small>
              </div>

              <div className="form-group">
                <label htmlFor="hardwareExperience">
                  Hardware Experience <span className="required">*</span>
                </label>
                <select
                  id="hardwareExperience"
                  name="hardwareExperience"
                  value={formData.hardwareExperience}
                  onChange={handleChange}
                  required
                >
                  <option value="">Select your hardware experience level</option>
                  <option value="none">None - New to hardware</option>
                  <option value="beginner">Beginner - Basic understanding</option>
                  <option value="intermediate">Intermediate - Some project experience</option>
                  <option value="advanced">Advanced - Regular hardware work</option>
                  <option value="expert">Expert - Hardware professional</option>
                </select>
                <small className="field-help">Helps determine depth of hardware-focused content</small>
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
                  <option value="other">Other GPU</option>
                </select>
                <small className="field-help">Knowing your hardware helps suggest appropriate exercises</small>
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
                <small className="field-help">Helps recommend resources based on memory capacity</small>
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
                <small className="field-help">Allows for OS-specific instructions and tools</small>
              </div>

              <div className="step-navigation">
                <button onClick={prevStep} disabled={loading} className="prev-step-btn">
                  ← Back
                </button>
                <button
                  onClick={handleSignup}
                  disabled={loading || !formData.softwareExperience || !formData.hardwareExperience}
                  className="finish-signup-btn"
                >
                  {loading ? 'Creating Account...' : 'Finish Sign Up'}
                </button>
              </div>
            </div>
          )}

          {error && <div className="error-banner">{error}</div>}
        </div>
      </div>
    </div>
  );
};

export default SignupModal;