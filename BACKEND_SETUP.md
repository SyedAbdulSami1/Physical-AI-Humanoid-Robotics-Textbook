# RAG Chatbot Backend Setup

This document provides step-by-step instructions to set up the RAG (Retrieval-Augmented Generation) chatbot backend for the Physical AI & Humanoid Robotics Textbook.

## Prerequisites

- Python 3.9 or higher
- Access to Neon Serverless Postgres (free tier)
- Access to Qdrant Cloud (free tier)
- OpenAI API key
- Node.js (for the frontend, if running locally)

## Setup Instructions

### 1. Environment Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd Physical-AI-Humanoid-Robotics-Textbook
   ```

2. Create a virtual environment and install dependencies:
   ```bash
   cd app
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Copy the environment variables template:
   ```bash
   cp .env.example .env
   ```

### 2. Neon Serverless Postgres Setup

1. Go to [Neon Console](https://console.neon.tech/)
2. Create a new project
3. Copy the connection string in the format: `postgresql+asyncpg://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require`
4. Add this to your `.env` file as `NEON_DATABASE_URL`

### 3. Qdrant Cloud Setup

1. Go to [Qdrant Cloud](https://cloud.qdrant.io/)
2. Create a new cluster
3. Get your cluster URL and API key
4. Add these to your `.env` file as `QDRANT_URL` and `QDRANT_API_KEY`

### 4. OpenAI API Setup

1. Go to [OpenAI Platform](https://platform.openai.com/)
2. Create an API key
3. Add this to your `.env` file as `OPENAI_API_KEY`

### 5. Run the Backend

1. Start the FastAPI server:
   ```bash
   cd app
   uvicorn main:app --reload --port 8000
   ```

2. The API will be available at `http://localhost:8000`

### 6. Run Ingestion Pipeline

1. First ensure your textbook content is in the `docs/` directory as Markdown files
2. To run the ingestion pipeline, make an authenticated request to the `/ingest/run` endpoint:
   ```bash
   curl -X POST "http://localhost:8000/ingest/run" \
     -H "Authorization: Bearer <your-jwt-token>" \
     -H "Content-Type: application/json"
   ```

### 7. Test the API Endpoints

#### Authentication
```bash
# Register a new user
curl -X POST "http://localhost:8000/auth/signup" \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "password123",
    "profile": {
      "role": "student",
      "technical_expertise": "beginner",
      "primary_interest": "Bipedal Locomotion"
    }
  }'

# Login
curl -X POST "http://localhost:8000/auth/login" \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d 'username=user@example.com&password=password123'
```

#### Chat with Full Book Context
```bash
curl -X POST "http://localhost:8000/chat/" \
  -H "Authorization: Bearer <your-jwt-token>" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?"
  }'
```

#### Chat with Selected Text Only
```bash
curl -X POST "http://localhost:8000/chat/" \
  -H "Authorization: Bearer <your-jwt-token>" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept",
    "selected_text": "Physical AI is a field that combines artificial intelligence with physical systems like robots..."
  }'
```

## API Endpoints

- `GET /` - Root endpoint to confirm the API is running
- `POST /auth/signup` - Register a new user
- `POST /auth/login` - Login and get JWT token
- `GET /auth/me` - Get current user profile
- `POST /ingest/run` - Trigger content ingestion (requires authentication)
- `POST /chat/` - Chat with the RAG system (requires authentication)
- `POST /personalize/` - Get personalized content (requires authentication)
- `POST /translate/` - Get translated content (requires authentication)

## Architecture

The backend is built with:
- FastAPI for the web framework with async support
- SQLAlchemy with asyncpg for Neon Postgres database interactions
- Qdrant for vector storage and similarity search
- OpenAI for text generation
- JWT for authentication
- LangChain for RAG operations

The RAG pipeline:
1. Chunks the textbook Markdown content
2. Creates vector embeddings using OpenAI
3. Stores embeddings in Qdrant
4. On query, embeds the user's question
5. Finds relevant content from the vector store
6. Combines the context with the question using a prompt
7. Generates a response using OpenAI
8. Returns the answer with source citations