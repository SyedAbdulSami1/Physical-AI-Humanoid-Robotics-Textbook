import React, { useState, useEffect, useRef } from 'react';
import { useUser } from '@auth0/nextjs-auth0/client';

const ChatbotComponent = ({ chapterSlug }) => {
  const [messages, setMessages] = useState([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);
  
  const { user, isLoading: userLoading } = useUser();

  // Function to get selected text
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText.length > 0) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSendMessage = async () => {
    if (!inputText.trim() || isLoading) return;

    // Add user message to chat
    const userMessage = { id: Date.now(), text: inputText, sender: 'user', timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    const currentInput = inputText;
    setInputText('');
    setIsLoading(true);

    try {
      // Prepare request body
      const requestBody = {
        message: currentInput,
        chapter_slug: chapterSlug,
        selected_text: selectedText || null
      };

      // Add user ID if available
      if (user) {
        requestBody.user_id = user.sub;
      }

      // Call backend API
      const response = await fetch('/api/v1/chat/message', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('access_token') || ''}`
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`Server error: ${response.status}`);
      }

      const data = await response.json();
      
      // Add bot response to chat
      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        sources: data.sources || [],
        timestamp: new Date()
      };
      
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>Physical AI & Humanoid Robotics Assistant</h3>
        {selectedText && (
          <div className="selected-text-indicator">
            Using selected text: "{selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}"
          </div>
        )}
      </div>
      
      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your Physical AI & Humanoid Robotics textbook assistant.</p>
            <p>Ask me anything about the content, or select text and ask specific questions about it.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div 
              key={message.id} 
              className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
            >
              <div className="message-content">
                {message.sender === 'bot' && message.sources && message.sources.length > 0 && (
                  <div className="sources">
                    <strong>Sources:</strong> {message.sources.slice(0, 2).join(', ')}
                  </div>
                )}
                {message.text}
              </div>
              <div className="timestamp">
                {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>
      
      <div className="chat-input-area">
        <textarea
          ref={textareaRef}
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about the content..."
          disabled={isLoading}
          rows="3"
        />
        <button 
          onClick={handleSendMessage} 
          disabled={!inputText.trim() || isLoading}
          className="send-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default ChatbotComponent;