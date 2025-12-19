# Setup script for Physical AI & Humanoid Robotics Textbook RAG Chatbot

#!/bin/bash
set -e  # Exit on any error

echo "==========================================="
echo "Physical AI & Humanoid Robotics Textbook Setup"
echo "==========================================="

# Check if Python 3.8+ is installed
if ! command -v python3 &> /dev/null; then
    echo "Python3 is not installed. Please install Python 3.8 or higher."
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
if [ "$(printf '%s\n' "3.8" "$PYTHON_VERSION" | sort -V | head -n1)" != "3.8" ]; then
    echo "Python version should be 3.8 or higher. Current version: $PYTHON_VERSION"
    exit 1
fi

echo "✓ Python version check passed: $PYTHON_VERSION"

# Check if pip is available
if ! command -v pip &> /dev/null; then
    echo "pip is not installed. Installing pip..."
    python3 -m ensurepip --upgrade
fi

echo "✓ pip is available"

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

echo "✓ Virtual environment ready"

# Activate virtual environment
source venv/bin/activate  # Use venv\Scripts\activate for Windows
echo "✓ Virtual environment activated"

# Upgrade pip
pip install --upgrade pip
echo "✓ pip upgraded"

# Install backend dependencies
echo "Installing backend dependencies..."
pip install -r app/requirements.txt
echo "✓ Backend dependencies installed"

# Install frontend dependencies if package.json exists
if [ -f "package.json" ]; then
    echo "Installing frontend dependencies..."
    npm install
    echo "✓ Frontend dependencies installed"
else
    echo "⚠ package.json not found, skipping frontend dependencies"
fi

# Create .env file if it doesn't exist
if [ ! -f "app/.env" ]; then
    echo "Creating .env file with default values in app/ directory..."
    cat > app/.env << EOF
# Security
AUTH_SECRET_KEY="your_auth_secret_key"

# Database Configuration
NEON_DATABASE_URL="your_neon_database_url"

# Qdrant Configuration
QDRANT_URL="your_qdrant_url"
QDRANT_API_KEY="your_qdrant_api_key"

# OpenAI Configuration
OPENAI_API_KEY="your_openai_api_key"
EOF
    echo "⚠ Created .env file in app/ with default values - please update with your actual credentials"
fi

echo "==========================================="
echo "Setup completed successfully!"
echo "==========================================="
echo ""
echo "To run the backend server:"
echo "  1. Activate virtual environment: source venv/bin/activate"
echo "  2. Run: python -m uvicorn app.main:app --reload"
echo ""
echo "To run the Docusaurus frontend:"
echo "  1. Run: npm start"
echo ""
echo "For production deployment:"
echo "  1. Update .env with production values"
echo "  2. Run: npm run build"
echo "  3. Deploy the build/ folder to your hosting service"