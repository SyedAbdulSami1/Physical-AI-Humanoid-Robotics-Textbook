from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, Dict, Any
import os
import uvicorn
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = FastAPI(title="Physical AI & Humanoid Robotics API", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
class User(BaseModel):
    email: str
    password: str
    name: str

class LoginRequest(BaseModel):
    email: str
    password: str

class UserProfile(BaseModel):
    softwareExperience: str
    hardwareExperience: str
    gpuType: str
    ramSize: str
    osType: str

class PersonalizeRequest(BaseModel):
    content: str
    user_profile: Dict[str, Any]

class TranslateRequest(BaseModel):
    text: str
    target_lang: str
    source_lang: str = "en"

# Mock data storage (in production, use a real database)
users_db = {}
user_profiles = {}

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics API is running!"}

@app.post("/auth/signup/")
async def signup(user: User):
    # In a real app, you would hash the password and store in DB
    if user.email in users_db:
        raise HTTPException(status_code=400, detail="User already registered")
    
    users_db[user.email] = {
        "id": len(users_db) + 1,
        "email": user.email,
        "password": user.password,  # In production, hash the password
        "name": user.name
    }
    
    return {"message": "User created successfully", "user_id": users_db[user.email]["id"]}

@app.post("/auth/login/")
async def login(login_request: LoginRequest):
    user = users_db.get(login_request.email)
    if not user or user["password"] != login_request.password:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    
    # In a real app, you would create a JWT token
    return {
        "message": "Login successful",
        "user": {"id": user["id"], "email": user["email"], "name": user["name"]},
        "token": f"mock_token_for_{user['email']}"
    }

@app.post("/api/user/profile/")
async def save_user_profile(profile: UserProfile, token: str = None):
    # In a real app, you would verify the token and extract user info
    # Here we'll just use a placeholder user_id
    user_id = 1  # This would come from the token in real implementation
    user_profiles[user_id] = profile.dict()
    
    return {"message": "Profile saved successfully", "user_id": user_id}

@app.get("/api/user/profile/{user_id}")
async def get_user_profile(user_id: int):
    profile = user_profiles.get(user_id)
    if not profile:
        raise HTTPException(status_code=404, detail="User profile not found")
    
    return profile

@app.post("/personalize/")
async def personalize_content(request: PersonalizeRequest):
    # This is where the personalization logic would go
    # For now, we'll return the content unchanged but in the future
    # this would adapt based on the user_profile
    content = request.content
    profile = request.user_profile
    
    # Placeholder logic - in real implementation, you'd adapt the content
    # based on user profile (experience level, hardware, etc.)
    if profile.get("softwareExperience") == "beginner":
        # Add extra explanations for beginners
        adapted_content = f"[Beginner-friendly version: {content}]"
    elif profile.get("softwareExperience") == "expert":
        # Add advanced details for experts
        adapted_content = f"[Advanced version: {content}]"
    else:
        adapted_content = content

    return {"personalized_content": adapted_content}

@app.post("/api/translate/")
async def translate_text(request: TranslateRequest):
    # This is where the translation logic would go
    # For now, we'll return the original text as a placeholder
    # In real implementation, integrate with LibreTranslate or Google Translate API
    
    # Placeholder translation - in real implementation, call translation API
    if request.target_lang == "ur":
        # For this example, we'll return the English text as is
        # In a real implementation, this would be translated to Urdu
        translated_text = f"[Translated to {request.target_lang}]: {request.text}"
    else:
        translated_text = request.text
    
    return {"translated_text": translated_text}

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    uvicorn.run("main:app", host="0.0.0.0", port=port, reload=True)