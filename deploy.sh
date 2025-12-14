#!/bin/bash

# This script deploys the Docusaurus frontend to Vercel.
# The backend is configured to deploy automatically on Render.com when changes are pushed to the main branch.

# --- Prerequisites ---
# 1. Ensure the Vercel CLI is installed: `npm install -g vercel`
# 2. Ensure you are logged into Vercel: `vercel login`
# 3. Ensure your project is linked to a Vercel project: `vercel link`

# --- Frontend Deployment (Vercel) ---
echo "Deploying frontend to Vercel..."

# The `vercel` command will build and deploy the Docusaurus site.
# The --prod flag creates a production deployment.
vercel --prod

if [ $? -eq 0 ]; then
  echo "✅ Frontend deployment successful!"
else
  echo "❌ Frontend deployment failed."
  exit 1
fi

# --- Backend Deployment (Render) ---
echo "Backend is configured for automatic deployment on Render.com."
echo "Push your changes to the main branch to trigger a new deployment."

exit 0
