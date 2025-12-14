// src/theme/Root.tsx

import React from 'react';
import Root from '@theme-original/Root';
import { SelectedTextProvider } from './Chatbot/SelectedTextProvider';
import Chatbot from './Chatbot/Chatbot';
import AuthForm from '@site/src/components/AuthForm'; // Assuming you might want a modal login
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

export default function RootWrapper({children}) {
  const [isLoggedIn, setIsLoggedIn] = React.useState(false);

  React.useEffect(() => {
    // Check for auth token on initial load
    const token = localStorage.getItem('authToken');
    if (token) {
      setIsLoggedIn(true);
    }
  }, []);

  const handleLoginSuccess = () => {
    setIsLoggedIn(true);
    // You might want to close a modal here if AuthForm is in one
  };

  return (
    <SelectedTextProvider>
      {/* The original Docusaurus root component that renders the page */}
      <Root>{children}</Root>

      {/* Render the chatbot on all pages */}
      <Chatbot />

      {/*
        This is a conceptual example of how you might integrate the other components.
        You would likely have a modal system or place these buttons in specific page layouts
        rather than rendering them on every single page like this.

        For example, the AuthForm could be in a modal triggered from the navbar.
        The Personalize/Translate buttons would be passed the chapterId from the specific doc page.
      */}
      {/* <div style={{ position: 'fixed', top: '100px', right: '20px', zIndex: 2000, background: 'white', padding: '1rem', border: '1px solid #ccc', borderRadius: '8px' }}>
        {!isLoggedIn ? (
          <AuthForm onLoginSuccess={handleLoginSuccess} />
        ) : (
          <div>
            <p>Welcome! You are logged in.</p>
            <PersonalizeButton chapterId="example-chapter-id" onContentPersonalized={(content) => console.log(content)} />
            <UrduTranslationButton chapterId="example-chapter-id" onContentTranslated={(content) => console.log(content)} />
          </div>
        )}
      </div> */}
    </SelectedTextProvider>
  );
}
