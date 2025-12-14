// src/theme/Chatbot/Chatbot.js
import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useSelectedText } from './SelectedTextProvider';
import ChatMessage from './ChatMessage';
import './chatbot.css';

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, sender: 'bot', text: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics textbook. Ask me anything about the content!' }
  ]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  
  const { selectedText } = useSelectedText();

  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages, scrollToBottom]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current.focus(), 100);
    }
  };

  const handleSend = async () => {
    if (input.trim() === '') return;

    const userMessage = { 
      id: Date.now(), 
      sender: 'user', 
      text: input,
      selectedText: selectedText // Include selected text if available
    };
    
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const token = localStorage.getItem('authToken');
      if (!token) {
        setMessages(prev => [...prev, { 
          id: Date.now(), 
          sender: 'bot', 
          text: 'You must be logged in to use the chat. Please sign up or log in first.' 
        }]);
        setLoading(false);
        return;
      }

      const requestBody = {
        query: input,
        selected_text: selectedText || null // Send selected text if available
      };

      const response = await fetch(`${API_URL}/chat/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();
      
      const botMessage = {
        id: Date.now(),
        sender: 'bot',
        text: data.answer,
        sources: data.sources
      };
      
      setMessages(prev => [...prev, botMessage]);

    } catch (error) {
      console.error("Chat API error:", error);
      const errorMessage = { 
        id: Date.now(), 
        sender: 'bot', 
        text: 'Sorry, I encountered an error processing your request. Please try again.' 
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setLoading(false);
      if (inputRef.current) {
        inputRef.current.focus();
      }
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div className="chatbot-container">
      <button className="chatbot-toggle-button" onClick={toggleChat}>
        {isOpen ? '×' : '🤖'}
      </button>
      
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <h3>AI Assistant</h3>
            <button className="chatbot-close-button" onClick={toggleChat} aria-label="Close chat">
              ×
            </button>
          </div>
          
          <div className="chatbot-messages">
            {messages.map((message) => (
              <ChatMessage key={message.id} message={message} />
            ))}
            
            {loading && (
              <div className="message bot-message">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          
          <div className="chatbot-input-form">
            {selectedText && (
              <div className="selected-text-indicator">
                <span className="selected-text-label">Using selected text:</span>
                <span className="selected-text-preview">"{selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}"</span>
              </div>
            )}
            <div className="input-area">
              <textarea
                ref={inputRef}
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder="Ask a question about the textbook..."
                className="chat-input"
                aria-label="Type your message"
              />
              <button 
                onClick={handleSend} 
                disabled={loading}
                className="send-button"
                aria-label="Send message"
              >
                {loading ? '...' : '➤'}
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;