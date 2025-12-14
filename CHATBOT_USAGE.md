# RAG Chatbot Frontend - Usage Instructions

## Overview

The RAG (Retrieval-Augmented Generation) chatbot is integrated into the Docusaurus textbook website. It allows users to ask questions about the textbook content and get AI-generated responses based on the material.

## Features

1. **Normal Chat Mode**: Ask general questions about the textbook content
2. **Selected-Text Mode**: Select specific text on the page and ask questions about only that content
3. **Markdown Rendering**: Responses with proper formatting, code blocks, and lists
4. **Source Citations**: Each response includes the sources used to generate the answer
5. **Mobile Responsive**: Works well on all device sizes
6. **Loading States**: Visual feedback during API requests

## How to Use

### Normal Chat
1. Click the chatbot icon (🤖) in the bottom right corner
2. Type your question in the input field
3. Press Enter or click the send button
4. The AI will respond with information based on the entire textbook

### Selected-Text Mode
1. Highlight any text on the page
2. Open the chatbot (the selected text will automatically appear as context)
3. Ask a question related to the selected text
4. The AI will respond using only the selected text as context

## UI Components

- **Floating Chat Button**: Toggles the chat interface
- **Message History**: Shows conversation with alternating user/bot messages
- **Input Area**: Text area for typing questions
- **Selected Text Indicator**: Shows when selected text is being used
- **Typing Indicator**: Shows when the AI is processing your request
- **Sources**: Lists the documents used to generate each response

## Technical Details

- **Backend Integration**: Connects to the FastAPI backend at `/chat/` endpoint
- **Authentication**: Requires JWT token stored in localStorage
- **Markdown Support**: Renders responses with full markdown support including code blocks
- **Context Awareness**: Automatically detects and includes selected text in API requests

## Testing Steps

1. Start the backend server: `cd app && uvicorn main:app --reload`
2. Start the frontend: `yarn start` (from project root)
3. Navigate to a textbook page
4. Test normal chat by asking a question
5. Test selected-text mode by selecting text and asking a question
6. Verify that sources are properly cited
7. Test on different screen sizes to ensure responsiveness

## Troubleshooting

- If the chat doesn't work, ensure the backend server is running
- Check that you're logged in (tokens stored in localStorage)
- Verify the API URL is correctly set in environment variables
- Make sure the ingestion pipeline has run successfully