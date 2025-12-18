import React, { useState, useEffect } from 'react';
import MDXContent from '@theme/MDXContent';
import TranslateButton from '@site/src/components/TranslateButton';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import { getChapterContent } from '@site/src/utils/contentUtils';

// This component wraps the doc content with personalization and translation buttons
export default function DocItemContent({ children }) {
  // Since we're in a static generation context, we'll implement a safe way to access auth
  // without requiring the AuthProvider context during build
  const getUserFromStorage = () => {
    if (typeof window !== 'undefined') {
      const storedUser = localStorage.getItem('user');
      return storedUser ? JSON.parse(storedUser) : null;
    }
    return null;
  };

  const [user, setUser] = useState(null);

  useEffect(() => {
    // Get user from localStorage in browser environment
    const storedUser = getUserFromStorage();
    setUser(storedUser);

    // Set up storage event listener to update user state when auth changes in another tab
    const handleStorageChange = () => {
      const updatedUser = getUserFromStorage();
      setUser(updatedUser);
    };

    window.addEventListener('storage', handleStorageChange);
    return () => window.removeEventListener('storage', handleStorageChange);
  }, []);

  const [translatedContent, setTranslatedContent] = useState(null);
  const [personalizedContent, setPersonalizedContent] = useState(null);
  const [currentContent, setCurrentContent] = useState(children);

  // Function to handle translation
  const handleTranslate = (translatedText) => {
    if (translatedText) {
      setTranslatedContent(translatedText);
      // When we have translation, we ignore personalization
      setCurrentContent(translatedText);
    } else {
      // Reset to original content
      setTranslatedContent(null);
      setPersonalizedContent(null);
      setCurrentContent(children);
    }
  };

  // Function to handle personalization
  const handlePersonalize = (personalizedText) => {
    if (personalizedText) {
      // If we have translation, don't override it with personalization
      if (!translatedContent) {
        setPersonalizedContent(personalizedText);
        setCurrentContent(personalizedText);
      }
    } else {
      // Reset to original content
      setPersonalizedContent(null);
      setCurrentContent(children);
    }
  };

  // Reset to original content when user changes or logs out
  useEffect(() => {
    if (!user) {
      setTranslatedContent(null);
      setPersonalizedContent(null);
      setCurrentContent(children);
    }
  }, [user, children]);

  // Create MDX components to provide the necessary context
  const MDXComponents = {
    // Add any specific MDX components that are expected
    // This might include Details, Tabs, etc. depending on your content
    details: (props) => <details {...props} />,
    // Add any other components that might be missing
  };

  return (
    <div className="container margin-vert--lg">
      <div className="chapter-controls">
        <div className="button-group">
          <PersonalizeButton
            onPersonalize={handlePersonalize}
          />
          <TranslateButton
            content={typeof children === 'string' ? children : JSON.stringify(children)}
            onTranslate={handleTranslate}
            language="ur"
          />
        </div>
      </div>

      <div className="doc-content">
        <MDXContent>
          {currentContent}
        </MDXContent>
      </div>
    </div>
  );
}