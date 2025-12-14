// src/theme/Chatbot/ChatMessage.js
import React from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

const ChatMessage = ({ message }) => {
  const { sender, text, sources } = message;

  return (
    <div className={`message ${sender}-message`}>
      <div className="message-content">
        <ReactMarkdown 
          remarkPlugins={[remarkGfm]} 
          className="markdown-content"
        >
          {text}
        </ReactMarkdown>
      </div>
      
      {sources && sources.length > 0 && (
        <div className="message-sources">
          <strong>Sources:</strong>
          <ul>
            {sources.map((source, index) => (
              <li key={index}>{source}</li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
};

export default ChatMessage;