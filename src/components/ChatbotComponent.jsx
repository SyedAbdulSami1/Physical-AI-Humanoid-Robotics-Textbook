// src/components/ChatbotComponent.jsx

import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import styles from './styles.css'; // We'll create this CSS file

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const ChatbotComponent = () => {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState([]);
    const [input, setInput] = useState('');
    const [loading, setLoading] = useState(false);
    const messagesEndRef = useRef(null);

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    };

    useEffect(scrollToBottom, [messages]);

    const toggleChat = () => {
        setIsOpen(!isOpen);
        if (!isOpen && messages.length === 0) {
            setMessages([
                { sender: 'bot', text: 'Hello! Ask me anything about Physical AI and Humanoid Robotics.' }
            ]);
        }
    };

    const handleSend = async () => {
        if (input.trim() === '') return;

        const userMessage = { sender: 'user', text: input };
        setMessages(prev => [...prev, userMessage]);
        setInput('');
        setLoading(true);

        try {
            // Assume token is stored in localStorage after login
            const token = localStorage.getItem('authToken');
            if (!token) {
                setMessages(prev => [...prev, { sender: 'bot', text: 'You must be logged in to use the chat.' }]);
                setLoading(false);
                return;
            }

            const response = await axios.post(
                `${API_URL}/chat/`,
                { query: input },
                { headers: { Authorization: `Bearer ${token}` } }
            );
            
            const botMessage = { 
                sender: 'bot', 
                text: response.data.answer,
                sources: response.data.sources 
            };
            setMessages(prev => [...prev, botMessage]);

        } catch (error) {
            const errorMessage = { sender: 'bot', text: 'Sorry, I encountered an error. Please try again.' };
            setMessages(prev => [...prev, errorMessage]);
            console.error("Chat API error:", error);
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="chatbot-container">
            <button className="chatbot-toggle-button" onClick={toggleChat}>
                {isOpen ? 'Close' : 'Chat'}
            </button>
            {isOpen && (
                <div className="chatbot-window">
                    <div className="chatbot-header">
                        <h2>AI Assistant</h2>
                        <button onClick={toggleChat} className="chatbot-close-button">X</button>
                    </div>
                    <div className="chatbot-messages">
                        {messages.map((msg, index) => (
                            <div key={index} className={`message ${msg.sender}`}>
                                <p>{msg.text}</p>
                                {msg.sources && (
                                    <div className="sources">
                                        <strong>Sources:</strong>
                                        <ul>
                                            {msg.sources.map((source, i) => (
                                                <li key={i}>{source}</li>
                                            ))}
                                        </ul>
                                    </div>
                                )}
                            </div>
                        ))}
                        {loading && <div className="message bot typing-indicator"><span>.</span><span>.</span><span>.</span></div>}
                        <div ref={messagesEndRef} />
                    </div>
                    <div className="chatbot-input-form">
                        <input
                            type="text"
                            value={input}
                            onChange={(e) => setInput(e.target.value)}
                            onKeyPress={(e) => e.key === 'Enter' && handleSend()}
                            placeholder="Ask a question..."
                        />
                        <button onClick={handleSend} disabled={loading}>Send</button>
                    </div>
                </div>
            )}
        </div>
    );
};

export default ChatbotComponent;
