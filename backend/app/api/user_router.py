from fastapi import APIRouter, Depends, HTTPException, status
from better_auth import auth
from better_auth.fastapi_plugin import get_current_user
from pydantic import BaseModel
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import database models
from ..db import get_db, User, Profile
from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError

# Create API router
user_router = APIRouter()

# Pydantic models
class ProfileUpdate(BaseModel):
    softwareExperience: Optional[str] = None
    hardwareExperience: Optional[str] = None

class ProfileResponse(BaseModel):
    id: str
    softwareExperience: Optional[str] = None
    hardwareExperience: Optional[str] = None
    userId: str

# Get user profile
@user_router.get("/profile/{user_id}", response_model=ProfileResponse)
async def get_user_profile(user_id: str, db: Session = Depends(get_db)):
    profile = db.query(Profile).filter(Profile.user_id == user_id).first()
    if not profile:
        # Return an empty profile if not found
        return ProfileResponse(
            id="", 
            softwareExperience=None, 
            hardwareExperience=None, 
            userId=user_id
        )
    return ProfileResponse(
        id=profile.id,
        softwareExperience=profile.software_experience,
        hardwareExperience=profile.hardware_experience,
        userId=profile.user_id
    )

# Update user profile
@user_router.post("/profile", response_model=ProfileResponse)
async def update_user_profile(
    profile_data: ProfileUpdate,
    current_user=Depends(get_current_user),
    db: Session = Depends(get_db)
):
    if not current_user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated"
        )
    
    # Check if profile already exists
    existing_profile = db.query(Profile).filter(Profile.user_id == current_user.id).first()
    
    if existing_profile:
        # Update existing profile
        existing_profile.software_experience = profile_data.softwareExperience
        existing_profile.hardware_experience = profile_data.hardwareExperience
        db.commit()
        db.refresh(existing_profile)
        profile = existing_profile
    else:
        # Create new profile
        profile = Profile(
            user_id=current_user.id,
            software_experience=profile_data.softwareExperience,
            hardware_experience=profile_data.hardwareExperience
        )
        db.add(profile)
        try:
            db.commit()
            db.refresh(profile)
        except IntegrityError:
            db.rollback()
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Profile already exists for this user"
            )
    
    return ProfileResponse(
        id=profile.id,
        softwareExperience=profile.software_experience,
        hardwareExperience=profile.hardware_experience,
        userId=profile.user_id
    )

# Get current user info
@user_router.get("/me")
async def get_current_user_info(current_user=Depends(get_current_user)):
    if not current_user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated"
        )
    return current_user