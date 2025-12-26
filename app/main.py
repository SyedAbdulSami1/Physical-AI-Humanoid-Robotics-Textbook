# app/main.py

import sys
import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# This allows the app to be run from the 'app' directory, making 'app' a discoverable package
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


from app.routers import ingest, chat



# Load environment variables from .env file
load_dotenv()

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
    # Add the production frontend URL here once deployed
    # "https://your-docusaurus-site.com",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Event Handlers ---





# --- API Routers ---


app.include_router(ingest.router)
app.include_router(chat.router)




@app.get("/", tags=["Root"])
async def read_root():
    """
    A simple endpoint to confirm the API is running.
    """
    return {"message": "Welcome to the Physical AI Textbook API!"}
