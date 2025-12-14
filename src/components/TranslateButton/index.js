import React, { useState } from 'react';

const TranslateButton = ({ content, onTranslate, language = 'ur' }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState(null);

  const handleTranslate = async () => {
    setIsTranslating(true);
    setError(null);

    try {
      // Call backend API to translate content to Urdu
      const response = await fetch('/api/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: content,
          targetLanguage: language
        }),
      });

      if (response.ok) {
        const { translatedText } = await response.json();
        onTranslate(translatedText);
      } else {
        const errorData = await response.json();
        setError(errorData.error || 'Translation failed');
        console.error('Translation API error:', errorData);
      }
    } catch (err) {
      setError('Network error occurred during translation');
      console.error('Translation error:', err);
    } finally {
      setIsTranslating(false);
    }
  };

  const handleReset = () => {
    onTranslate(null); // Reset to original content
    setError(null);
  };

  return (
    <div className="translate-controls">
      {onTranslate && (
        <button
          onClick={isTranslating ? handleReset : handleTranslate}
          disabled={isTranslating}
          className={`translate-btn ${isTranslating ? 'loading' : ''}`}
        >
          {isTranslating ? 'Translating...' : 'Translate to Urdu'}
        </button>
      )}
      {error && <div className="error">{error}</div>}
    </div>
  );
};

export default TranslateButton;