# Backend Setup Instructions

This document provides instructions for setting up the backend services for the Physical AI & Humanoid Robotics Textbook, including Better-Auth and Neon Postgres. Note that translation services have been removed to maintain 100% free operation of the project.

## Prerequisites

Before setting up the backend, ensure you have the following:

- Node.js (v18 or higher)
- Python (v3.9 or higher)
- PostgreSQL client or access to a Neon Postgres database
- Git

## Setup Steps

### 1. Clone the Repository

```bash
git clone <your-repo-url>
cd Physical-AI-Humanoid-Robotics-Textbook
```

### 2. Backend Setup

#### 2.1. Navigate to Backend Directory

```bash
cd backend
```

#### 2.2. Create and Activate Virtual Environment

```bash
python -m venv venv
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate
```

#### 2.3. Install Dependencies

```bash
pip install -r requirements.txt
```

#### 2.4. Environment Configuration

Create a `.env` file in the `backend` directory with the following content:

```env
# Database Configuration
DATABASE_URL="postgresql://username:password@localhost:5432/textbook"

# Better-Auth Configuration
AUTH_SECRET="your-super-secret-key-change-this-in-production"
```

> **Important**: For production use, always use strong, randomly generated secrets and store them securely using environment variables or a secrets management system.

### 3. Database Setup with Neon Postgres

#### 3.1. Create a Neon Account

1. Go to [Neon Console](https://console.neon.tech/)
2. Sign up or log in to your account
3. Create a new project

#### 3.2. Get Connection Details

1. In your Neon project dashboard, go to the "Connection Details" section
2. Copy the connection string and update your `.env` file:

```env
DATABASE_URL="postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/textbook?sslmode=require"
```

#### 3.3. Run Database Migrations

The backend will create necessary tables automatically on startup. Just start the server as described below.

# Authentication Removed

Authentication has been removed from the project to maintain 100% free operation. The RAG chatbot is now fully public with no authentication required.



### 6. Running the Backend Server



#### 6.1. Start the Backend Server



```bash

cd backend

# Make sure your virtual environment is activated

python main.py

```



Alternatively, use uvicorn directly:



```bash

cd backend

uvicorn main:app --reload --host 0.0.0.0 --port 8000

```



The backend will be available at `http://localhost:8000`.



### 7. Frontend Setup



#### 7.1. Install Node.js Dependencies



```bash

# From the project root

npm install

```



#### 7.2. Run the Docusaurus Frontend



```bash

# From the project root

npm start

```



The frontend will be available at `http://localhost:3000`.



## API Endpoints



### RAG Chatbot Endpoints



- `POST /chat/` - Chat with the RAG bot using textbook content

- `POST /chat/` with selected text - Chat with the RAG bot focusing on selected text only

- `GET /health` - Health check endpoint

- `POST /ingest/run` - Ingest textbook content into vector database



## Troubleshooting



### Common Issues



1. **Database Connection Issues**: 

   - Ensure your Neon Postgres connection string is correct

   - Check that your IP address is allowed in Neon settings

   - Verify SSL settings



2. **Better-Auth Issues**:

   - Ensure `AUTH_SECRET` has sufficient entropy (at least 32 characters)

   - Check that the database is correctly configured



3. **Frontend-Backend Connection**:

   - By default, the frontend expects the backend at `http://localhost:8000`

   - If running on a different port, update the API calls accordingly

### Development Tips

1. **Hot Reload**: Backend server supports hot reload with `--reload` flag
2. **CORS**: The backend allows all origins by default; restrict this in production
3. **Logging**: Check backend logs for detailed error information
4. **Frontend**: Use the browser's developer tools to inspect API requests

## Security Considerations

- Never commit secrets to the repository
- Use strong, randomly generated secrets for `AUTH_SECRET`
- Restrict database access to only necessary IPs
- Enable SSL/TLS for all connections
- Use environment variables for all configuration
- Implement rate limiting for API endpoints in production