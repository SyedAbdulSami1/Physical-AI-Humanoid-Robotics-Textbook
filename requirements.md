# Website

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

---

# Project Setup and Execution

This project consists of a Docusaurus frontend and a Python (FastAPI) backend. Follow these steps to set up and run the full application locally.

## 1. Prerequisites

-   [Node.js](https://nodejs.org/en/) (v18 or higher)
-   [Yarn](https://yarnpkg.com/)
-   [Python](https://www.python.org/downloads/) (v3.9 or higher)

## 2. Environment Variables

Create a `.env` file in the root of the project directory. This file is crucial for storing all your secrets and API keys.

```env
# .env

# Backend - Neon Serverless Postgres
# Get this from your Neon dashboard
NEON_DATABASE_URL="postgresql+asyncpg://<user>:<password>@<host>/<dbname>"

# Backend - Qdrant Cloud
# Get these from your Qdrant Cloud dashboard
QDRANT_URL="https"
QDRANT_API_KEY=""

# Backend - OpenAI
# Get this from your OpenAI dashboard
OPENAI_API_KEY="sk-..."

# Backend - Authentication
# Use a long, random string for security
AUTH_SECRET_KEY="a_very_secret_key_that_should_be_long_and_random"

# Frontend (Optional - if your deployed API is not at localhost:8000)
# REACT_APP_API_URL="http://your-backend-api-url.com"
```

## 3. Backend Setup

Navigate to the `app` directory and set up a Python virtual environment.

```bash
# From the project root
cd app

# Create a virtual environment
python -m venv venv

# Activate the virtual environment
# On Windows
venv\Scripts\activate
# On macOS/Linux
source venv/bin/activate

# Install the required Python packages
pip install -r requirements.txt
```

## 4. Frontend Setup

Install the Node.js dependencies using Yarn.

```bash
# From the project root
yarn install
```

## 5. Running the Application

You need to run the backend and frontend servers in two separate terminals.

**Terminal 1: Start the Backend Server**

```bash
# From the project root, inside the app directory with venv active
cd app
uvicorn main:app --reload
```

The backend API will be running at `http://localhost:8000`.

**Terminal 2: Start the Frontend Server**

```bash
# From the project root
yarn start
```

The Docusaurus website will be running at `http://localhost:3000`.

## 6. Content Ingestion

After running the backend for the first time, you must populate the vector database with the textbook content. You can do this by sending a POST request to the `/ingest/run` endpoint.

**Note**: This requires a valid authentication token.

1.  Go to the website (`http://localhost:3000`).
2.  Create an account using the Sign-Up form.
3.  Log in with your new account.
4.  Use a tool like Postman, `curl`, or a simple script to send a POST request to `http://localhost:8000/ingest/run` with your authentication token in the `Authorization` header (`Bearer <your_token>`).

Once the ingestion is complete, the chatbot will be able to answer questions about the textbook.

# Project Requirements

This project is a documentation website built using Docusaurus.

## Features
- Book-style documentation
- Sidebar navigation
- Responsive design

## Tools Used
- Docusaurus
- Node.js
- GitHub

## Installation
npm install
npm run start
