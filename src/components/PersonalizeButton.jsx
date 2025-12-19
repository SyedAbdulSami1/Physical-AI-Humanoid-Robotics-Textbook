// src/components/PersonalizeButton.jsx

import React, { useState, useEffect } from 'react';
import axios from 'axios';
import styles from './styles.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const PersonalizeButton = ({ children, onPersonalize }) => {
    const { siteConfig } = useDocusaurusContext();
    const { apiUrl } = siteConfig.customFields;

    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');
    const [userProfile, setUserProfile] = useState(null);
    const [isAuthenticated, setIsAuthenticated] = useState(false);

    useEffect(() => {
        // Check authentication and load user profile on component mount
        const token = localStorage.getItem('better-auth.session_token');
        setIsAuthenticated(!!token);

        if (token) {
            // Load user profile from localStorage or fetch from backend
            const profile = JSON.parse(localStorage.getItem('userProfile') || '{}');
            setUserProfile(profile);
        }
    }, []);

    const handlePersonalize = async () => {
        const token = localStorage.getItem('better-auth.session_token');

        if (!token) {
            setError('You must be logged in to personalize content.');
            return;
        }

        if (!userProfile || Object.keys(userProfile).length === 0) {
            setError('Please complete your profile to enable personalization.');
            return;
        }

        setLoading(true);
        setError('');

        try {
            // Get the raw HTML or text content to personalize
            let contentToPersonalize = '';
            if (typeof children === 'string') {
                contentToPersonalize = children;
            } else if (React.isValidElement(children)) {
                // If it's a React element, we need to extract text content
                contentToPersonalize = children.props.children || '';
            } else {
                // Handle other cases
                contentToPersonalize = '';
            }

            // Call personalization API with both the content and user profile
            const response = await axios.post(
                `${apiUrl}/personalize/`,
                {
                    content: contentToPersonalize,
                    user_profile: userProfile
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

    if (!isAuthenticated) {
        return null; // Only show the button if user is authenticated
    }

    return (
        <div className="feature-button-container">
            <button
                onClick={handlePersonalize}
                disabled={loading}
                className="feature-button"
                title="Adapt content to your experience level"
            >
                {loading ? (
                    <span>🔄 Personalizing...</span>
                ) : (
                    <span>✨ Personalize Content</span>
                )}
            </button>
            {error && <p className="error-message">{error}</p>}
        </div>
    );
};

export default PersonalizeButton;
