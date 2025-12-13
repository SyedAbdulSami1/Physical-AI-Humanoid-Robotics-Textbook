from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text, Boolean, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID
import uuid
import os
from dotenv import load_dotenv

load_dotenv()

# Database URL for Neon Postgres
DATABASE_URL = os.getenv(
    "NEON_DATABASE_URL", 
    "postgresql://username:password@ep-dry-snowflake-123456.us-east-1.aws.neon.tech/dbname?sslmode=require"
)

# Create engine and session
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# Dependency to get database session
def get_db_dependency():
    db = SessionLocal()
    try:
        return db
    except Exception:
        db.close()