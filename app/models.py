from sqlalchemy import Column, Integer, String, DateTime, Text, Boolean, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from app.database import Base
import uuid
from sqlalchemy.dialects.postgresql import UUID

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String, unique=True, index=True, nullable=False)
    name = Column(String, nullable=True)
    hashed_password = Column(String, nullable=False)  # Added for authentication
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationships
    sessions = relationship("Session", back_populates="user")
    user_preferences = relationship("UserPreference", back_populates="user")

class UserPreference(Base):
    __tablename__ = "user_preferences"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"))
    background_level = Column(String, nullable=False)  # novice, intermediate, advanced
    programming_language = Column(String, nullable=True)  # python, c++, etc.
    hardware_interest = Column(String, nullable=True)  # humanoid robots, industrial, etc.
    learning_goal = Column(Text, nullable=True)  # specific goals
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationships
    user = relationship("User", back_populates="user_preferences")

class Session(Base):
    __tablename__ = "sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"))
    session_token = Column(String, unique=True, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    expires_at = Column(DateTime(timezone=True), nullable=False)
    is_active = Column(Boolean, default=True)

    # Relationships
    user = relationship("User", back_populates="sessions")
    chat_histories = relationship("ChatHistory", back_populates="session")

class ChatHistory(Base):
    __tablename__ = "chat_history"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("sessions.id"))
    user_input = Column(Text, nullable=False)
    bot_response = Column(Text, nullable=False)
    chapter_context = Column(String, nullable=True)  # Chapter where the query was made
    selected_text = Column(Text, nullable=True)  # Selected text for selected-text-only mode
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    # Relationships
    session = relationship("Session", back_populates="chat_histories")

class BookChapter(Base):
    __tablename__ = "book_chapters"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String, nullable=False)
    slug = Column(String, unique=True, nullable=False)  # URL-friendly identifier
    content = Column(Text, nullable=False)  # Full chapter content
    vector_embedding_id = Column(String, nullable=True)  # Reference to Qdrant embedding ID
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

class PersonalizedContent(Base):
    __tablename__ = "personalized_content"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chapter_id = Column(UUID(as_uuid=True), ForeignKey("book_chapters.id"))
    user_preference_id = Column(UUID(as_uuid=True), ForeignKey("user_preferences.id"))
    content_version = Column(Text, nullable=False)  # Personalized version of content
    personalization_level = Column(String, nullable=False)  # novice, intermediate, advanced
    created_at = Column(DateTime(timezone=True), server_default=func.now())