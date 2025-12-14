import React, { useState, useEffect } from 'react';
import { ThemeClassNames } from '@docusaurus/theme-common';
import { useDoc } from '@docusaurus/use-doc';
import TranslateButton from '@site/src/components/TranslateButton';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import { useAuth } from '../Auth/Auth';
import { getChapterContent } from '@site/src/utils/contentUtils';

// This component wraps the doc content with personalization and translation buttons
export default function DocItemContent({ children }) {
  const { frontMatter, metadata } = useDoc();
  const { user } = useAuth();

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

  return (
    <div className={ThemeClassNames.doc.docMainContainer}>
      <div className="personalization-translation-controls">
        <div className="button-group">
          <PersonalizeButton
            chapterContent={children}
            onPersonalize={handlePersonalize}
          />

          <TranslateButton
            content={children}
            onTranslate={handleTranslate}
            language="ur"
          />
        </div>
      </div>

      <div className="doc-content">
        {currentContent}
      </div>
    </div>
  );
}