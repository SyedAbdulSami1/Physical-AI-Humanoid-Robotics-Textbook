# app/database.py

import os
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from dotenv import load_dotenv
from pathlib import Path

# --- .env Loading ---
# This ensures that the DATABASE_URL is loaded before it's used.
# It's good practice to have this in the module where the connection is defined.
dotenv_path = Path(__file__).resolve().parent / '.env'
load_dotenv(dotenv_path=dotenv_path)

# --- Database URL ---
# Get the connection string from the environment variables.
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")

if not NEON_DATABASE_URL:
    raise ValueError("NEON_DATABASE_URL environment variable not set. Please check your .env file.")

# --- SQLAlchemy Engine Setup ---
# The engine is the starting point for any SQLAlchemy application.
# It's the 'home base' for the actual database and its DBAPI.
# We use an async engine for FastAPI compatibility.
engine = create_async_engine(
    NEON_DATABASE_URL,
    echo=True,  # Set to False in production for cleaner logs
)

# --- Session Maker ---
# The sessionmaker factory generates new Session objects when called.
# This is our primary interface for database communication.
SessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=engine,
    class_=AsyncSession, # Use AsyncSession for async operations
)

# --- Dependency for FastAPI Routes ---
async def get_db() -> AsyncSession:
    """
    FastAPI dependency that provides a database session for each request.
    It ensures that the session is always closed after the request is finished.
    """
    async with SessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()