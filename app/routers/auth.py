# app/routers/auth.py

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordRequestForm
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select

from app import models, schemas
from app.database import get_db
from app.auth import get_password_hash, verify_password, create_access_token

router = APIRouter(
    prefix="/auth",
    tags=["Authentication"],
)

@router.post("/signup", response_model=schemas.User)
async def signup(user_data: schemas.UserCreate, db: AsyncSession = Depends(get_db)):
    """
    Register a new user and their profile information.
    """
    # Check if user already exists
    result = await db.execute(select(models.User).where(models.User.email == user_data.email))
    db_user = result.scalar_one_or_none()
    if db_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered",
        )

    # Hash the password
    hashed_password = get_password_hash(user_data.password)

    # Create new user and profile
    new_user = models.User(
        email=user_data.email,
        hashed_password=hashed_password,
    )
    
    new_profile = models.UserProfile(
        user=new_user,
        role=user_data.profile.role,
        technical_expertise=user_data.profile.technical_expertise,
        primary_interest=user_data.profile.primary_interest,
        has_nvidia_gpu=user_data.profile.has_nvidia_gpu,
        gpu_model=user_data.profile.gpu_model,
        has_jetson_device=user_data.profile.has_jetson_device,
        jetson_model=user_data.profile.jetson_model,
    )

    db.add(new_user)
    db.add(new_profile)
    await db.commit()
    await db.refresh(new_user)

    return new_user

@router.post("/login", response_model=schemas.Token)
async def login(form_data: OAuth2PasswordRequestForm = Depends(), db: AsyncSession = Depends(get_db)):
    """
    Log in a user and return an access token.
    """
    result = await db.execute(select(models.User).where(models.User.email == form_data.username))
    user = result.scalar_one_or_none()

    if not user or not verify_password(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Create access token
    access_token = create_access_token(data={"sub": user.email})

    return {"access_token": access_token, "token_type": "bearer"}

@router.get("/me", response_model=schemas.User)
async def read_users_me(current_user: models.User = Depends(schemas.get_current_active_user)):
    """
    Get the profile of the currently authenticated user.
    """
    return current_user
