# app/models.py

import enum
from sqlalchemy import (
    create_engine,
    Column,
    Integer,
    String,
    ForeignKey,
    DateTime,
    Boolean,
    Enum,
    Text
)
from sqlalchemy.orm import relationship, sessionmaker
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class UserRole(enum.Enum):
    STUDENT = "student"
    PROFESSIONAL = "professional"
    HOBBYIST = "hobbyist"
    RESEARCHER = "researcher"

class TechnicalExpertise(enum.Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"
    EXPERT = "expert"

class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    hashed_password = Column(String, nullable=False)
    is_active = Column(Boolean, default=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    profile = relationship("UserProfile", back_populates="user", uselist=False, cascade="all, delete-orphan")

class UserProfile(Base):
    __tablename__ = "user_profiles"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    
    # User Background Questionnaire
    role = Column(Enum(UserRole), nullable=True)
    technical_expertise = Column(Enum(TechnicalExpertise), nullable=True)
    primary_interest = Column(String, nullable=True) # e.g., "Bipedal Locomotion", "Computer Vision", "LLM Integration"
    
    # Hardware Background
    has_nvidia_gpu = Column(Boolean, default=False)
    gpu_model = Column(String, nullable=True)
    has_jetson_device = Column(Boolean, default=False)
    jetson_model = Column(String, nullable=True)

    # Personalization settings
    preferred_explanation_style = Column(Enum(TechnicalExpertise), default=TechnicalExpertise.BEGINNER)
    
    user = relationship("User", back_populates="profile")

# Example of how to create the engine and session
# This will be done in database.py, but shown here for context

# DATABASE_URL = "postgresql+asyncpg://user:password@host/dbname"
# engine = create_engine(DATABASE_URL)
# SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# To create tables:
# Base.metadata.create_all(bind=engine)
