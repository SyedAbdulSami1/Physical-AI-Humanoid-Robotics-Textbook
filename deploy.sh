# Deployment script for Physical AI & Humanoid Robotics Textbook

#!/bin/bash
set -e  # Exit on any error

echo "==========================================="
echo "Physical AI & Humanoid Robotics Textbook Deployment"
echo "==========================================="

DEPLOYMENT_TYPE=$1

if [ "$DEPLOYMENT_TYPE" != "vercel" ] && [ "$DEPLOYMENT_TYPE" != "github-pages" ] && [ "$DEPLOYMENT_TYPE" != "local" ]; then
    echo "Usage: ./deploy.sh [vercel|github-pages|local]"
    exit 1
fi

case $DEPLOYMENT_TYPE in
    "vercel")
        echo "Deploying to Vercel..."
        
        # Check if Vercel CLI is installed
        if ! command -v vercel &> /dev/null; then
            echo "Vercel CLI is not installed. Installing..."
            npm install -g vercel
        fi
        
        # Build the Docusaurus site
        echo "Building Docusaurus site..."
        npm run build
        
        # Deploy to Vercel
        echo "Deploying to Vercel..."
        vercel --prod --token=$VERCEL_TOKEN
        
        echo "✓ Deployed to Vercel successfully!"
        ;;
        
    "github-pages")
        echo "Deploying to GitHub Pages..."
        
        # Build the Docusaurus site
        echo "Building Docusaurus site..."
        npm run build
        
        # Deploy to GitHub Pages
        echo "Deploying to GitHub Pages..."
        npx gh-pages -d build -t true
        
        echo "✓ Deployed to GitHub Pages successfully!"
        ;;
        
    "local")
        echo "Setting up local development environment..."
        
        # Make sure backend is running
        echo "Starting backend server..."
        echo "Run 'python -m uvicorn app.main:app --reload' in another terminal"
        
        # Start frontend in development mode
        echo "Starting Docusaurus development server..."
        npm start
        
        echo "✓ Local development environment ready!"
        echo "Frontend should be accessible at http://localhost:3000"
        ;;
esac

echo "==========================================="
echo "Deployment completed!"
echo "==========================================="