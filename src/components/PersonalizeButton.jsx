// src/components/PersonalizeButton.jsx

import React, { useState } from 'react';
import axios from 'axios';
import styles from './styles.css';

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const PersonalizeButton = ({ chapterId, onContentPersonalized }) => {
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');

    const handlePersonalize = async () => {
        setLoading(true);
        setError('');

        try {
            const token = localStorage.getItem('authToken');
            if (!token) {
                setError('You must be logged in to personalize content.');
                setLoading(false);
                return;
            }

            const response = await axios.post(
                `${API_URL}/personalize/`,
                { chapter_id: chapterId },
                { headers: { Authorization: `Bearer ${token}` } }
            );
            
            if (onContentPersonalized) {
                onContentPersonalized(response.data.personalized_content);
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
                {loading ? 'Personalizing...' : '✨ Personalize This Chapter'}
            </button>
            {error && <p className="error-message">{error}</p>}
        </div>
    );
};

export default PersonalizeButton;
