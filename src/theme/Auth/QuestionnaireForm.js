import React, { useState, useEffect } from 'react';
import { useAuth } from '../Auth/Auth';

const QuestionnaireForm = () => {
  const { user, isPending } = useAuth();
  const [formData, setFormData] = useState({
    softwareExperience: '',
    hardwareExperience: '',
    gpuType: '',
    ramSize: '',
    osType: ''
  });
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState(false);

  useEffect(() => {
    if (user && user.id) {
      // Load existing questionnaire data if available
      fetchQuestionnaireData();
    }
  }, [user]);

  const fetchQuestionnaireData = async () => {
    if (!user?.id) return;

    try {
      setLoading(true);
      const response = await fetch(`/api/user/profile/${user.id}`, {
        headers: {
          'Authorization': `Bearer ${user.token || user.sessionToken}` // Adjust to actual token property
        }
      });

      if (response.ok) {
        const data = await response.json();
        setFormData(data);
      }
    } catch (err) {
      console.error('Error fetching questionnaire data:', err);
      setError('Failed to load questionnaire data');
    } finally {
      setLoading(false);
    }
  };

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setSuccess(false);

    try {
      const response = await fetch('/api/user/profile', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${user.token || user.sessionToken}` // Adjust to actual token property
        },
        body: JSON.stringify(formData)
      });

      if (!response.ok) {
        throw new Error('Failed to save questionnaire data');
      }

      setSuccess(true);
      setTimeout(() => setSuccess(false), 3000);
    } catch (err) {
      setError('An error occurred while saving the questionnaire');
      console.error(err);
    }
  };

  if (isPending) return <div>Loading...</div>;
  if (!user) return <div>Please sign in to access the questionnaire.</div>;
  if (loading) return <div>Loading questionnaire...</div>;

  return (
    <div className="questionnaire-form">
      <h3>User Profile & Experience</h3>
      {error && <div className="error">{error}</div>}
      {success && <div className="success">Profile updated successfully!</div>}
      
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="softwareExperience">Software Experience</label>
          <select
            id="softwareExperience"
            name="softwareExperience"
            value={formData.softwareExperience}
            onChange={handleChange}
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
        
        <button type="submit">Save Profile</button>
      </form>
    </div>
  );
};

export default QuestionnaireForm;