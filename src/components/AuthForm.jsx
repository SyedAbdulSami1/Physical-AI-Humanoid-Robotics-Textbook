// src/components/AuthForm.jsx

import React, { useState } from 'react';
import axios from 'axios';
import styles from './styles.css';

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const AuthForm = ({ onLoginSuccess }) => {
    const [isLogin, setIsLogin] = useState(true);
    const [error, setError] = useState('');
    const [loading, setLoading] = useState(false);

    // Form fields
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [profile, setProfile] = useState({
        role: 'STUDENT',
        technical_expertise: 'BEGINNER',
        primary_interest: '',
        has_nvidia_gpu: false,
        gpu_model: '',
        has_jetson_device: false,
        jetson_model: '',
    });

    const handleProfileChange = (e) => {
        const { name, value, type, checked } = e.target;
        setProfile(prev => ({
            ...prev,
            [name]: type === 'checkbox' ? checked : value
        }));
    };

    const handleSubmit = async (e) => {
        e.preventDefault();
        setLoading(true);
        setError('');

        if (isLogin) {
            // --- Login Logic ---
            try {
                const formData = new URLSearchParams();
                formData.append('username', email);
                formData.append('password', password);

                const response = await axios.post(`${API_URL}/auth/login`, formData, {
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' }
                });

                localStorage.setItem('authToken', response.data.access_token);
                if (onLoginSuccess) onLoginSuccess();
                
            } catch (err) {
                setError(err.response?.data?.detail || 'Login failed. Please check your credentials.');
            }
        } else {
            // --- Signup Logic ---
            try {
                const signupData = {
                    email,
                    password,
                    profile,
                };
                await axios.post(`${API_URL}/auth/signup`, signupData);
                // Switch to login tab and notify user
                setIsLogin(true);
                alert('Signup successful! Please log in.');

            } catch (err) {
                setError(err.response?.data?.detail || 'Signup failed. Please try again.');
            }
        }
        setLoading(false);
    };

    return (
        <div className="auth-form-container">
            <div className="auth-form-tabs">
                <button onClick={() => setIsLogin(true)} className={isLogin ? 'active' : ''}>Log In</button>
                <button onClick={() => setIsLogin(false)} className={!isLogin ? 'active' : ''}>Sign Up</button>
            </div>
            <form onSubmit={handleSubmit} className="auth-form">
                <h2>{isLogin ? 'Log In' : 'Create Your Account'}</h2>
                
                <div className="form-group">
                    <label>Email</label>
                    <input type="email" value={email} onChange={(e) => setEmail(e.target.value)} required />
                </div>
                <div className="form-group">
                    <label>Password</label>
                    <input type="password" value={password} onChange={(e) => setPassword(e.target.value)} required />
                </div>

                {!isLogin && (
                    <>
                        <hr/>
                        <h3>Tell Us About Yourself</h3>
                        <div className="form-group">
                            <label>Role</label>
                            <select name="role" value={profile.role} onChange={handleProfileChange}>
                                <option value="STUDENT">Student</option>
                                <option value="PROFESSIONAL">Professional</option>
                                <option value="HOBBYIST">Hobbyist</option>
                                <option value="RESEARCHER">Researcher</option>
                            </select>
                        </div>
                        <div className="form-group">
                            <label>Technical Expertise</label>
                            <select name="technical_expertise" value={profile.technical_expertise} onChange={handleProfileChange}>
                                <option value="BEGINNER">Beginner</option>
                                <option value="INTERMEDIATE">Intermediate</option>
                                <option value="ADVANCED">Advanced</option>
                                <option value="EXPERT">Expert</option>
                            </select>
                        </div>
                         <div className="form-group">
                            <label>Primary Interest</label>
                            <input type="text" name="primary_interest" value={profile.primary_interest} onChange={handleProfileChange} placeholder="e.g., Robot Vision" />
                        </div>
                         <div className="form-group checkbox-group">
                            <label>Do you have an NVIDIA GPU?</label>
                            <input type="checkbox" name="has_nvidia_gpu" checked={profile.has_nvidia_gpu} onChange={handleProfileChange} />
                        </div>
                        {profile.has_nvidia_gpu && (
                             <div className="form-group">
                                <label>GPU Model</label>
                                <input type="text" name="gpu_model" value={profile.gpu_model} onChange={handleProfileChange} placeholder="e.g., RTX 4090" />
                            </div>
                        )}
                    </>
                )}

                {error && <p className="error-message">{error}</p>}

                <button type="submit" disabled={loading} className="submit-button">
                    {loading ? 'Processing...' : (isLogin ? 'Log In' : 'Sign Up')}
                </button>
            </form>
        </div>
    );
};

export default AuthForm;
