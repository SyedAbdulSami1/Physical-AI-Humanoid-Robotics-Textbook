import React, { useState } from 'react';

const UrduTranslationButton = ({ chapterContent, chapterTitle }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [error, setError] = useState(null);
  const [viewMode, setViewMode] = useState('english'); // 'english', 'urdu', 'bilingual'

  const translateContent = async () => {
    setIsTranslating(true);
    setError(null);
    
    try {
      // In a real implementation, this would call our backend translation API
      // Here we'll simulate the API call
      const response = await fetch('/api/v1/translation/urdu', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          text: chapterContent,
          target_lang: 'ur'
        })
      });
      
      if (!response.ok) {
        throw new Error(`Translation API error: ${response.status}`);
      }
      
      const data = await response.json();
      setTranslatedContent(data.translated_text);
      setViewMode('urdu');
    } catch (err) {
      console.error('Translation error:', err);
      setError('Translation failed. Please try again later.');
    } finally {
      setIsTranslating(false);
    }
  };

  const toggleViewMode = (mode) => {
    setViewMode(mode);
  };

  return (
    <div className="urdu-translation-container">
      <div className="translation-controls">
        <button 
          onClick={translateContent}
          disabled={isTranslating || viewMode === 'urdu'}
          className={`translate-button ${viewMode === 'urdu' ? 'active' : ''}`}
        >
          {isTranslating ? 'Translating...' : 
           viewMode === 'urdu' ? ' Urdu ✓' : 
           'Translate to Urdu'}
        </button>
        
        {translatedContent && (
          <div className="view-mode-selector">
            <button 
              onClick={() => toggleViewMode('english')}
              className={viewMode === 'english' ? 'active' : ''}
            >
              English
            </button>
            <button 
              onClick={() => toggleViewMode('urdu')}
              className={viewMode === 'urdu' ? 'active' : ''}
            >
              Urdu
            </button>
            <button 
              onClick={() => toggleViewMode('bilingual')}
              className={viewMode === 'bilingual' ? 'active' : ''}
            >
              Bilingual
            </button>
          </div>
        )}
      </div>
      
      {error && (
        <div className="translation-error">
          {error}
        </div>
      )}
      
      <div className="translation-content">
        {viewMode === 'english' && (
          <div className="english-content">
            {chapterContent}
          </div>
        )}
        
        {viewMode === 'urdu' && (
          <div className="urdu-content" dir="rtl">
            {translatedContent ? (
              <div>{translatedContent}</div>
            ) : (
              <div className="translation-placeholder">
                {isTranslating 
                  ? 'Translating content to Urdu...' 
                  : 'Click "Translate to Urdu" to see content in Urdu'}
              </div>
            )}
          </div>
        )}
        
        {viewMode === 'bilingual' && (
          <div className="bilingual-content">
            <div className="english-section">
              <h4>English</h4>
              <div>{chapterContent}</div>
            </div>
            <div className="urdu-section" dir="rtl">
              <h4>اردو</h4>
              {translatedContent ? (
                <div>{translatedContent}</div>
              ) : (
                <div className="translation-placeholder">
                  {isTranslating 
                    ? 'Translating content to Urdu...' 
                    : 'Translation in progress...'}
                </div>
              )}
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default UrduTranslationButton;