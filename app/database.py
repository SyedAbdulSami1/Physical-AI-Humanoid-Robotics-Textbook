# app/database.py

import os
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from dotenv import load_dotenv
from app.models import Base  # Import Base from models.py

# Load environment variables from .env file
load_dotenv()

# Get the database URL from environment variables
# The URL should be in the format: "postgresql+asyncpg://<user>:<password>@<host>/<dbname>"
DATABASE_URL = os.getenv("NEON_DATABASE_URL")

if not DATABASE_URL:
    raise ValueError("NEON_DATABASE_URL environment variable is not set. Please create a .env file and add it.")

# Create an asynchronous engine
engine = create_async_engine(DATABASE_URL, echo=True)

# Create a configured "Session" class
AsyncSessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=engine,
    class_=AsyncSession,
    expire_on_commit=False,
)

# Dependency to get a DB session
async def get_db():
    """
    Dependency that provides a database session for a single request.
    Ensures the session is properly closed after the request is completed.
    """
    async with AsyncSessionLocal() as session:
        yield session

async def init_db():
    """
    Initializes the database by creating all tables defined in the models.
    This should be called once when the application starts up.
    """
    async with engine.begin() as conn:
        # This will create all tables. In a production environment,
        # you would use a migration tool like Alembic.
        # await conn.run_sync(Base.metadata.drop_all) # Use with caution
        await conn.run_sync(Base.metadata.create_all)

# To run this and create the tables, you could have a small async script:
# import asyncio
# from app.database import init_db
#
# if __name__ == "__main__":
#     asyncio.run(init_db())
