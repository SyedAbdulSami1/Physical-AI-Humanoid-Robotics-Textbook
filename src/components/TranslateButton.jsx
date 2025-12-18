// src/components/TranslateButton.jsx

import React, { useState, useEffect } from 'react';
import { translateText } from '@site/src/utils/translationAPI';
import styles from './styles.css';

const TranslateButton = ({ content, onTranslate, language = 'ur' }) => {
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');
    const [isAuthenticated, setIsAuthenticated] = useState(false);

    useEffect(() => {
        // Check if user is authenticated
        const token = localStorage.getItem('better-auth.session_token');
        setIsAuthenticated(!!token);
    }, []);

    const handleTranslate = async () => {
        const token = localStorage.getItem('better-auth.session_token');

        if (!token) {
            setError('You must be logged in to translate content.');
            return;
        }

        if (!content) {
            setError('No content to translate.');
            return;
        }

        setLoading(true);
        setError('');

        try {
            // Use the translation API utility function
            const translatedText = await translateText(content, language, 'en');

            if (onTranslate) {
                onTranslate(translatedText);
            }
        } catch (err) {
            setError('Failed to translate content. Please try again later.');
            console.error("Translation API error:", err);
        } finally {
            setLoading(false);
        }
    };

    if (!isAuthenticated) {
        return null; // Only show the button if user is authenticated
    }

    return (
        <div className="feature-button-container">
            <button
                onClick={handleTranslate}
                disabled={loading}
                className="feature-button"
                title="Translate content to Urdu"
            >
                {loading ? (
                    <span>🔄 Translating...</span>
                ) : (
                    <span>🌐 Translate to Urdu</span>
                )}
            </button>
            {error && <p className="error-message">{error}</p>}
        </div>
    );
};

export default TranslateButton;