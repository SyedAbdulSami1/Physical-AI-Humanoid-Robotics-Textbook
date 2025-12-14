from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import Better-Auth
import better_auth
from better_auth import auth
from better_auth.fastapi_plugin import get_current_user

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="Backend API for the textbook with authentication, personalization, and translation features",
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

# Initialize Better-Auth
auth_instance = auth.Auth(
    secret=os.getenv("AUTH_SECRET", "default_secret_key_change_in_production"),
    database_url=os.getenv("DATABASE_URL"),
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

# Personalization endpoint
@app.post("/api/personalize", response_model=PersonalizeResponse)
async def personalize_content(
    request: PersonalizeRequest,
    current_user=Depends(get_current_user)
):
    from app.skills.orchestrator import skill_orchestrator

    if current_user.id != request.userId:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to personalize content for this user"
        )

    try:
        personalized_content = await skill_orchestrator.execute_skill(
            'personalization',
            request.content,
            request.profile
        )
        return PersonalizeResponse(personalizedContent=personalized_content)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Personalization failed: {str(e)}")

# Include auth router
@app.get("/api/auth/session")
async def get_session(current_user=Depends(get_current_user)):
    if not current_user:
        raise HTTPException(status_code=401, detail="Not authenticated")
    return current_user

# Include API routes
from app.api import user_router, rag_router
app.include_router(user_router, prefix="/api/user", tags=["user"])
app.include_router(rag_router, prefix="/api/rag", tags=["rag"])

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)