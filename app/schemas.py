# app/schemas.py

from pydantic import BaseModel, EmailStr, Field
from typing import Optional, List
from datetime import datetime
from app.models import UserRole, TechnicalExpertise

# --- Token Schemas ---

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    email: Optional[EmailStr] = None

# --- User and Profile Schemas ---

class UserProfileBase(BaseModel):
    role: Optional[UserRole] = None
    technical_expertise: Optional[TechnicalExpertise] = None
    primary_interest: Optional[str] = Field(None, max_length=100)
    has_nvidia_gpu: Optional[bool] = False
    gpu_model: Optional[str] = Field(None, max_length=50)
    has_jetson_device: Optional[bool] = False
    jetson_model: Optional[str] = Field(None, max_length=50)
    preferred_explanation_style: Optional[TechnicalExpertise] = TechnicalExpertise.BEGINNER

class UserProfileCreate(UserProfileBase):
    pass

class UserProfile(UserProfileBase):
    id: int

    class Config:
        orm_mode = True

class UserBase(BaseModel):
    email: EmailStr

class UserCreate(UserBase):
    password: str
    profile: UserProfileCreate

class User(UserBase):
    id: int
    is_active: bool
    created_at: datetime
    profile: UserProfile

    class Config:
        orm_mode = True

# --- Chat Schemas ---

class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    sources: List[str]

# --- Personalization Schemas ---

class PersonalizeRequest(BaseModel):
    chapter_id: str

class PersonalizeResponse(BaseModel):
    personalized_content: str

# --- Translation Schemas ---

class TranslationRequest(BaseModel):
    chapter_id: str

class TranslationResponse(BaseModel):
    translated_content: str

# --- Security and Dependency Injection ---

from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select

from app import auth, models
from app.database import get_db

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/auth/login")

async def get_current_user(token: str = Depends(oauth2_scheme), db: AsyncSession = Depends(get_db)) -> models.User:
    """
    Decodes the access token, validates the user, and returns the user object.
    Raises an exception if the token is invalid or the user doesn't exist.
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    payload = auth.decode_access_token(token)
    if payload is None:
        raise credentials_exception
    email: str = payload.get("sub")
    if email is None:
        raise credentials_exception
    
    result = await db.execute(select(models.User).where(models.User.email == email))
    user = result.scalar_one_or_none()

    if user is None:
        raise credentials_exception
    return user

async def get_current_active_user(current_user: models.User = Depends(get_current_user)) -> models.User:
    """
    Checks if the current user is active.
    Raises an exception if the user is inactive.
    """
    if not current_user.is_active:
        raise HTTPException(status_code=400, detail="Inactive user")
    return current_user
