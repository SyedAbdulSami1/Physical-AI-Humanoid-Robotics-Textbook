from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import JWTError, jwt
from datetime import datetime, timedelta
import os
from typing import Optional
from sqlalchemy.orm import Session
from app.models import User, UserPreference, Session as UserSession
from app.database import get_db
from passlib.context import CryptContext
import uuid
from pydantic import BaseModel
from enum import Enum

# Initialize password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security = HTTPBearer()

# Secret key for JWT tokens
SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-secret-key-here")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

class UserRegistrationRequest(BaseModel):
    email: str
    name: str
    password: str

class UserLoginRequest(BaseModel):
    email: str
    password: str

class UserQuestionnaireRequest(BaseModel):
    background_level: str  # "novice", "intermediate", "advanced"
    programming_language: Optional[str] = None
    hardware_interest: Optional[str] = None
    learning_goal: Optional[str] = None

class TokenData(BaseModel):
    user_id: str

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against a hashed password."""
    return pwd_context.verify(plain_password, hashed_password)

def hash_password(password: str) -> str:
    """Hash a plain password."""
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """Create a JWT token with expiration."""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def decode_token(token: str) -> Optional[TokenData]:
    """Decode a JWT token and return the payload."""
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id: str = payload.get("sub")
        if user_id is None:
            return None
        token_data = TokenData(user_id=user_id)
        return token_data
    except JWTError:
        return None

def authenticate_user(db: Session, email: str, password: str) -> Optional[User]:
    """Authenticate a user by email and password."""
    user = db.query(User).filter(User.email == email).first()
    if not user or not verify_password(password, user.hashed_password):
        return None
    return user

def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db)
) -> User:
    """Get the current authenticated user from the token."""
    token = credentials.credentials
    token_data = decode_token(token)
    if token_data is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
    user = db.query(User).filter(User.id == token_data.user_id).first()
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return user

def create_user(db: Session, email: str, name: str, password: str) -> User:
    """Create a new user with hashed password."""
    hashed_pwd = hash_password(password)
    
    # Check if user already exists
    existing_user = db.query(User).filter(User.email == email).first()
    if existing_user:
        raise ValueError("User with this email already exists")
    
    user = User(
        id=uuid.uuid4(),
        email=email,
        name=name,
        hashed_password=hashed_pwd
    )
    db.add(user)
    db.commit()
    db.refresh(user)
    return user

def create_user_session(db: Session, user_id: str) -> UserSession:
    """Create a new session for a user."""
    from datetime import datetime, timedelta
    session_token = create_access_token(data={"sub": user_id})
    expires_at = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    
    session = UserSession(
        id=uuid.uuid4(),
        user_id=user_id,
        session_token=session_token,
        expires_at=expires_at
    )
    db.add(session)
    db.commit()
    db.refresh(session)
    return session

def store_user_questionnaire(db: Session, user_id: str, questionnaire_data: UserQuestionnaireRequest) -> UserPreference:
    """Store user questionnaire response for personalization."""
    user_pref = UserPreference(
        id=uuid.uuid4(),
        user_id=user_id,
        background_level=questionnaire_data.background_level,
        programming_language=questionnaire_data.programming_language,
        hardware_interest=questionnaire_data.hardware_interest,
        learning_goal=questionnaire_data.learning_goal
    )
    db.add(user_pref)
    db.commit()
    db.refresh(user_pref)
    return user_pref

def get_user_preferences(db: Session, user_id: str) -> Optional[UserPreference]:
    """Retrieve user preferences for personalization."""
    return db.query(UserPreference).filter(UserPreference.user_id == user_id).first()