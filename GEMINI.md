You are a senior full-stack engineer and technical documentation agent working on a large open-source project called 
"Physical AI & Humanoid Robotics Textbook".

The project stack includes:
- Docusaurus (React) for the book and UI
- FastAPI (Python) backend
- A RAG chatbot
- Qdrant for vector search
- NeonDB (Postgres) for metadata

This project follows spec-driven development. You must read and respect the specification files before changing anything.

Your task is to implement the following four requirements strictly and completely:



────────────────────────────
2. REMOVE ALL AUTHENTICATION FROM THE RAG CHATBOT
────────────────────────────
- The chatbot must be fully public.
- Remove login, signup, sessions, JWT, middleware, guards, or protected routes related to the chatbot.
- Users must be able to ask questions immediately without any authentication.
- Update both backend and frontend accordingly.

────────────────────────────
3. IMPROVE AND MODERNIZE THE UI
────────────────────────────
- Improve the Docusaurus UI to be clean, modern, readable, and professional.
- Improve typography, layout, spacing, and navigation.
- Improve the chatbot UI (message bubbles, responsiveness, dark/light support if applicable).
- The UI should feel like a high-quality technical textbook platform.

────────────────────────────
4. UPDATE ALL SPEC AND PLANNING FILES
────────────────────────────
Update and reflect all changes in:
- spec.md
- tasks.md
- plan.md
- constitution.md

Each of these files must clearly state:
- The project uses only the free Google Gemini API.
- The chatbot has no authentication and is public.
- The UI has been modernized.

Mark relevant tasks as completed where appropriate.

────────────────────────────
RULES:
────────────────────────────
- Follow spec-driven development: read specs first, then implement.
- Do not invent new requirements.
- Do not introduce paid services, user accounts, or authentication.
- Do not hallucinate APIs.
- Make minimal, clean, production-ready changes.
- Prefer deterministic, maintainable code.

OUTPUT REQUIREMENTS:
- Show modified files or diffs clearly.
- Output updated versions of spec.md, tasks.md, plan.md, and constitution.md.

Start by analyzing the project structure, then implement all four requirements fully.

All Important key and api
# Security
AUTH_SECRET_KEY="755bac82c6fc810d7baaac0526bd23680e974130725de7e8c33d4365dfd58d34"

# Database Configuration
NEON_DATABASE_URL="postgresql://neondb_owner:npg_iktLCd2mREJ6@ep-delicate-sun-ahne97xv-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"

# Qdrant Configuration
QDRANT_URL="https://2e707ff9-216c-4a4d-a466-edc75be0a675.us-east4-0.gcp.cloud.qdrant.io"
QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.ak6OZ2qIGErAOaNWWYsIc518qxna24pX1MdAlaR36zg"

# Gemini Configuration
GEMINI_API_KEY="AIzaSyC1wJfcDuSl201ncYi30DPGyDVHoA7onZw"

# Docusaurus & Backend Configuration
# Replace the placeholder values with your actual credentials.

# --- Qdrant Vector Database ---
# The URL of your Qdrant instance.
QDRANT_URL="https://2e707ff9-216c-4a4d-a466-edc75be0a675.us-east4-0.gcp.cloud.qdrant.io"
# Your Qdrant API key (if required).
QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.ak6OZ2qIGErAOaNWWYsIc518qxna24pX1MdAlaR36zg"




# --- PostgreSQL Database (Neon) ---
# The connection string for your Neon serverless Postgres database.
NEON_DATABASE_URL="postgresql+asyncpg://neondb_owner:npg_iktLCd2mREJ6@ep-delicate-sun-ahne97xv-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"



# Backend Configuration
PORT=8000
NEON_DATABASE_URL="postgresql://neondb_owner:npg_iktLCd2mREJ6@ep-delicate-sun-ahne97xv-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"


# API Keys
GEMINI_API_KEY="AIzaSyC1wJfcDuSl201ncYi30DPGyDVHoA7onZw"


# Frontend URL (for CORS)
FRONTEND_URL=http://localhost:3000
