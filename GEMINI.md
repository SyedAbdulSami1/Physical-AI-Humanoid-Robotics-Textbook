You are a senior full-stack engineer and technical documentation agent working on a large open-source project called 
"Physical AI & Humanoid Robotics Textbook".

The project stack includes:
- Docusaurus (React) for the book and UI
- FastAPI (Python) backend
- A RAG chatbot
- Qdrant for vector search
- NeonDB (Postgres) for metadata

This project follows spec-driven development. You must read and respect the specification files before changing anything.

Your task is to implement the following five requirements strictly and completely:

────────────────────────────
1. USE ONLY FREE GOOGLE GEMINI API
────────────────────────────
- The entire project must use only the free Google Gemini API.
- No paid APIs, services, or features are allowed.
- Remove any references to paid or restricted services.

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
- No paid features are included.

Mark relevant tasks as completed where appropriate.

────────────────────────────
5. DOCUMENT REMOVAL OF TRANSLATION FEATURE
────────────────────────────
- Previously, the project had a translation feature (e.g., Urdu/English translation for content and chatbot).
- This feature was removed because it relied on paid APIs or services.
- Ensure no trace of translation functionality remains in code, UI, or documentation.
- Clearly document in all spec and planning files that translation was removed to keep the project 100% free.

────────────────────────────
RULES
────────────────────────────
- Follow spec-driven development: read specs first, then implement.
- Do not invent new requirements.
- Do not introduce paid services, user accounts, authentication, or paid features.
- Do not hallucinate APIs.
- Make minimal, clean, production-ready changes.
- Prefer deterministic, maintainable code.

OUTPUT REQUIREMENTS
- Show modified files or diffs clearly.
- Output updated versions of spec.md, tasks.md, plan.md, and constitution.md.

Start by analyzing the project structure, then implement all five requirements fully.


All Important key and api ("plz check in all .env files")
# Database Configuration
NEON_DATABASE_URL="plz check in .env"

# Qdrant Configuration
QDRANT_URL=""plz check in .env""
QDRANT_API_KEY=""plz check in .env""

# Gemini Configuration
GEMINI_API_KEY=""plz check in .env""

# Docusaurus & Backend Configuration
# Replace the placeholder values with your actual credentials.

# --- Qdrant Vector Database ---
# The URL of your Qdrant instance.
QDRANT_URL=""plz check in .env""
# Your Qdrant API key (if required).
QDRANT_API_KEY=""plz check in .env""




# --- PostgreSQL Database (Neon) ---
# The connection string for your Neon serverless Postgres database.
NEON_DATABASE_URL=""plz check in .env""



# Backend Configuration
PORT=8000
NEON_DATABASE_URL=""plz check in .env""


# API Keys
GEMINI_API_KEY=""plz check in .env""


# Frontend URL (for CORS)
FRONTEND_URL=http://localhost:3000
 Kal dobara koshish karen. Kal jab aap curl -X POST http://localhost:8000/ingest/run command chalayenge, to aapki
      API limit reset ho chuki hogi aur yeh command poori tarah chal jayegi.