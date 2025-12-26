from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="Backend API for the textbook with personalization, and translation features",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
class TranslationRequest(BaseModel):
    text: str
    targetLanguage: str = "ur"

class TranslationResponse(BaseModel):
    translatedText: str

class PersonalizeRequest(BaseModel):
    content: str
    profile: dict
    userId: str

class PersonalizeResponse(BaseModel):
    personalizedContent: str

# Health check endpoint
@app.get("/health")
async def health_check():
    return {"status": "ok"}

# Translation endpoint
@app.post("/api/translate", response_model=TranslationResponse)
async def translate_text(request: TranslationRequest):
    from app.services.translation_service import translate_text_to_urdu

    try:
        translated_text = await translate_text_to_urdu(request.text, request.targetLanguage)
        return TranslationResponse(translatedText=translated_text)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")

# Include API routes
from app.api import rag_router
app.include_router(rag_router, prefix="/api/rag", tags=["rag"])

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)