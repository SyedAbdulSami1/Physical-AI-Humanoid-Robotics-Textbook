@echo off
setlocal enabledelayedexpansion

echo ===========================================
echo Physical AI & Humanoid Robotics Textbook Setup
echo ===========================================

REM Check if Python 3.8+ is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo Python is not installed. Please install Python 3.8 or higher.
    pause
    exit /b 1
)

for /f "tokens=2" %%i in ('python --version 2^>^&1') do set PYTHON_VERSION=%%i
echo Python version: !PYTHON_VERSION!

REM Create virtual environment if it doesn't exist
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
)

echo Virtual environment ready

REM Activate virtual environment
call venv\Scripts\activate.bat
echo Virtual environment activated

REM Upgrade pip
python -m pip install --upgrade pip
echo pip upgraded

REM Install backend dependencies
if exist "app\requirements.txt" (
    echo Installing backend dependencies...
    pip install -r app\requirements.txt
    echo Backend dependencies installed
) else (
    echo app\requirements.txt not found!
)

REM Install frontend dependencies if package.json exists
if exist "package.json" (
    echo Installing frontend dependencies...
    npm install
    echo Frontend dependencies installed
) else (
    echo package.json not found, skipping frontend dependencies
)

REM Create .env file if it doesn't exist
if not exist ".env" (
    echo Creating .env file with default values...
    (
        echo # Database Configuration
        echo NEON_DATABASE_URL=postgresql://username:password@ep-dry-snowflake-123456.us-east-1.aws.neon.tech/dbname?sslmode=require
        echo.
        echo # Qdrant Configuration
        echo QDRANT_URL=https://your-cluster-url.europe-west3-0.gcp.cloud.qdrant.io:6333
        echo QDRANT_API_KEY=your-qdrant-api-key
        echo QDRANT_COLLECTION_NAME=textbook_content
        echo.
        
        echo GEMINI_API_KEY=your_gemini_api_key
        echo.
        echo # JWT Configuration
        echo JWT_SECRET_KEY=your-super-secret-jwt-key-change-in-production
        echo.
        echo # Application Configuration
        echo HOST=0.0.0.0
        echo PORT=8000
    ) > .env
    echo Created .env file with default values - please update with your actual credentials
)

echo ===========================================
echo Setup completed successfully!
echo ===========================================
echo.
echo To run the backend server:
echo   1. This environment is already activated
echo   2. Run: python -m uvicorn app.main:app --reload
echo.
echo To run the Docusaurus frontend:
echo   1. Run: npm start
echo.
echo For production deployment:
echo   1. Update .env with production values
echo   2. Run: npm run build
echo   3. Deploy the build^ folder to your hosting service

pause