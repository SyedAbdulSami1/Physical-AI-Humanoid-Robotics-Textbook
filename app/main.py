# app/main.py

import sys
import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import logging

# This allows the app to be run from the 'app' directory, making 'app' a discoverable package
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Load environment variables from .env file, overriding system variables
dotenv_path = os.path.join(os.path.dirname(__file__), '.env')
load_dotenv(dotenv_path=dotenv_path, override=True)

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from app.routers import ingest, chat

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="API for the interactive, AI-powered textbook.",
    version="1.0.0",
)

# --- Middleware ---

# Set up CORS (Cross-Origin Resource Sharing)
# This allows the Docusaurus frontend to communicate with the backend.
origins = [
    "http://localhost:3000",  # Docusaurus local dev server
    "http://127.0.0.1:3000",  # Alternative localhost for Docusaurus
    "http://localhost:8000",  # Self-origin for direct API calls
    # Add the production frontend URL here once deployed
    # "https://your-docusaurus-site.com",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,  # Allow specific origins instead of ["*"] for security
    allow_credentials=True,  # Allow credentials to be sent with requests
    allow_methods=["*"],     # Allow all methods (GET, POST, etc.)
    allow_headers=["*"],     # Allow all headers
)

# --- API Routers ---

app.include_router(ingest.router)
app.include_router(chat.router)

@app.get("/", tags=["Root"])
async def read_root():
    """
    A simple endpoint to confirm the API is running.
    """
    return {"message": "Welcome to the Physical AI Textbook API!"}

@app.get("/health", tags=["Health"])
async def health_check():
    """
    Health check endpoint to verify the API is running.
    """
    return {"status": "healthy", "message": "API is running"}
