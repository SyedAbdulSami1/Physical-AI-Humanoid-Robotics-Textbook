// src/components/UrduTranslationButton.jsx

import React, { useState } from 'react';
import axios from 'axios';
import styles from './styles.css';

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const UrduTranslationButton = ({ chapterId, onContentTranslated }) => {
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');

    const handleTranslate = async () => {
        setLoading(true);
        setError('');

        try {
            const token = localStorage.getItem('authToken');
            if (!token) {
                setError('You must be logged in to translate content.');
                setLoading(false);
                return;
            }

            const response = await axios.post(
                `${API_URL}/translate/urdu`,
                { chapter_id: chapterId },
                { headers: { Authorization: `Bearer ${token}` } }
            );
            
            if (onContentTranslated) {
                onContentTranslated(response.data.translated_content);
            }

        } catch (err) {
            setError(err.response?.data?.detail || 'Failed to translate content.');
            console.error("Translation API error:", err);
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="feature-button-container">
            <button 
                onClick={handleTranslate} 
                disabled={loading}
                className="feature-button"
            >
                {loading ? 'Translating...' : '🌐 Translate to Urdu'}
            </button>
            {error && <p className="error-message">{error}</p>}
        </div>
    );
};

export default UrduTranslationButton;
