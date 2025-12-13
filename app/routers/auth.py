from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel
import os
from app.database import get_db
from app.auth import (
    UserRegistrationRequest, 
    UserLoginRequest, 
    UserQuestionnaireRequest,
    create_user,
    authenticate_user,
    create_access_token,
    ACCESS_TOKEN_EXPIRE_MINUTES,
    get_current_user,
    store_user_questionnaire,
    get_user_preferences
)
from app.models import User
from datetime import timedelta
from typing import Optional

router = APIRouter()

class UserResponse(BaseModel):
    id: str
    email: str
    name: str

class Token(BaseModel):
    access_token: str
    token_type: str

class AuthResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: UserResponse

@router.post("/register", response_model=AuthResponse)
def register_user(request: UserRegistrationRequest, db: Session = Depends(get_db)):
    """
    Register a new user
    """
    try:
        user = create_user(
            db=db,
            email=request.email,
            name=request.name,
            password=request.password
        )
        
        # Create access token
        access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        access_token = create_access_token(
            data={"sub": str(user.id)}, expires_delta=access_token_expires
        )
        
        return AuthResponse(
            access_token=access_token,
            token_type="bearer",
            user=UserResponse(
                id=str(user.id),
                email=user.email,
                name=user.name
            )
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Registration failed: {str(e)}"
        )

@router.post("/login", response_model=AuthResponse)
def login_user(request: UserLoginRequest, db: Session = Depends(get_db)):
    """
    Authenticate user and return access token
    """
    try:
        user = authenticate_user(
            db=db,
            email=request.email,
            password=request.password
        )
        
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        # Create access token
        access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        access_token = create_access_token(
            data={"sub": str(user.id)}, expires_delta=access_token_expires
        )
        
        return AuthResponse(
            access_token=access_token,
            token_type="bearer",
            user=UserResponse(
                id=str(user.id),
                email=user.email,
                name=user.name
            )
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Login failed: {str(e)}"
        )

@router.post("/questionnaire")
def store_questionnaire(
    request: UserQuestionnaireRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Store user questionnaire response for personalization
    """
    try:
        user_preference = store_user_questionnaire(
            db=db,
            user_id=str(current_user.id),
            questionnaire_data=request
        )
        
        return {
            "message": "Questionnaire submitted successfully",
            "preference_id": str(user_preference.id)
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to store questionnaire: {str(e)}"
        )

@router.get("/me", response_model=UserResponse)
def get_user_profile(current_user: User = Depends(get_current_user)):
    """
    Get current user's profile
    """
    return UserResponse(
        id=str(current_user.id),
        email=current_user.email,
        name=current_user.name
    )

@router.get("/questionnaire")
def get_user_questionnaire(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Get user's questionnaire responses
    """
    try:
        user_prefs = get_user_preferences(db=db, user_id=str(current_user.id))
        if not user_prefs:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="No questionnaire found for this user"
            )
        
        return {
            "background_level": user_prefs.background_level,
            "programming_language": user_prefs.programming_language,
            "hardware_interest": user_prefs.hardware_interest,
            "learning_goal": user_prefs.learning_goal
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to retrieve questionnaire: {str(e)}"
        )