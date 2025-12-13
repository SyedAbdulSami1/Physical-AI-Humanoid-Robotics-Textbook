from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import routers
from app.routers import chat, ingest, auth

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup logic can go here
    yield
    # Shutdown logic can go here

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook RAG Chatbot",
    description="RAG-powered chatbot for the Physical AI & Humanoid Robotics textbook with personalized content adaptation",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure this properly
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router, prefix="/api/v1/chat", tags=["chat"])
app.include_router(ingest.router, prefix="/api/v1/ingest", tags=["ingest"])
app.include_router(auth.router, prefix="/api/v1/auth", tags=["auth"])

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics Textbook RAG Chatbot API"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host=os.getenv("HOST", "0.0.0.0"),
        port=int(os.getenv("PORT", 8000)),
        reload=True
    )