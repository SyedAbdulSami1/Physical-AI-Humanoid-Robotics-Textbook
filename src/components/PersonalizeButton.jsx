// src/components/PersonalizeButton.jsx

import React, { useState } from 'react';
import axios from 'axios';
import styles from './styles.css';

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const PersonalizeButton = ({ chapterContent, onPersonalize }) => {
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');

    const handlePersonalize = async () => {
        // Check if user is authenticated by looking for auth token in localStorage
        const token = localStorage.getItem('better-auth.session_token');

        if (!token) {
            setError('You must be logged in to personalize content.');
            return;
        }

        setLoading(true);
        setError('');

        try {
            // Get user's profile data to use for personalization
            // For now, we'll retrieve it from localStorage if available
            // In a real implementation, you'd fetch it from your backend
            const profileData = JSON.parse(localStorage.getItem('userProfile') || '{}');

            // Call personalization API with both the content and user profile
            const response = await axios.post(
                `${API_URL}/personalize/`,
                {
                    content: chapterContent,
                    user_profile: profileData
                },
                {
                    headers: {
                        'Authorization': `Bearer ${token}`,
                        'Content-Type': 'application/json'
                    }
                }
            );

            if (onPersonalize) {
                onPersonalize(response.data.personalized_content);
            }

        } catch (err) {
            setError(err.response?.data?.detail || 'Failed to personalize content.');
            console.error("Personalization API error:", err);
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="feature-button-container">
            <button
                onClick={handlePersonalize}
                disabled={loading}
                className="feature-button"
            >
                {loading ? 'Personalizing...' : '✨ Personalize This Content'}
            </button>
            {error && <p className="error-message">{error}</p>}
        </div>
    );
};

export default PersonalizeButton;
