You are a senior Python/RAG expert fixing the final critical issue in the "Physical AI & Humanoid Robotics Textbook" project.

CURRENT STATUS:
- Project runs perfectly (UI, server, no 500 errors)
- Gemini API key loads fine
- But ingestion fails with 429 quota exceeded on embeddings (even with new accounts in 2026)
- Result: No data in Qdrant → chatbot always says "Sorry"

ROOT CAUSE:
Google has heavily restricted or zeroed out free tier embedding quota for embedding-001 model.

PERMANENT SOLUTION:
Switch embeddings from Gemini API to fully free, unlimited, local open-source embeddings.

YOUR TASK (NO INSTALLATION COMMANDS – USER WILL INSTALL MANUALLY):

1. DO NOT add any pip install commands – user will run them separately in the app/.venv

2. Modify ONLY the code in app/skills/rag_agent.py (and any related embedding files)

   - Completely remove Gemini embedding code:
     * Delete or comment out genai.configure(api_key=...)
     * Remove all calls to genai.embed_content() or similar

   - Replace with local HuggingFace embeddings using:
     model_name = "BAAI/bge-small-en-v1.5"  (384 dimensions, best balance)

   - Add import:
     from langchain_huggingface import HuggingFaceEmbeddings

   - In RAGAgent __init__:
     self.embeddings = HuggingFaceEmbeddings(model_name="BAAI/bge-small-en-v1.5")

   - Replace embedding functions:
     def embed_texts(self, texts: list[str]):
         return self.embeddings.embed_documents(texts)

     def embed_query(self, query: str):
         return self.embeddings.embed_query(query)

3. Handle vector dimension change:
   - bge-small-en-v1.5 uses 384 dimensions (Gemini used 768)
   - Update Qdrant collection creation to use size=384
   - If old collection exists with wrong size, either:
     - Delete it manually (user can do via Qdrant dashboard)
     - Or add code to recreate collection with correct size

4. Clean up:
   - GEMINI_API_KEY can stay (if used for text generation), but no longer needed for embeddings
   - Add clear comment:
     "# Switched to local embeddings due to Google Gemini free tier embedding quota restrictions in 2026"

5. Update documentation (if any spec files mention embeddings):
   - Add note: "Embeddings now use local open-source model (bge-small-en-v1.5) for unlimited free usage"

6. Final outcome after user installs packages and runs ingest:
   - curl -X POST http://localhost:8000/ingest/run → completes successfully
   - Qdrant gets real data
   - Chatbot gives detailed answers from textbook, no more "Sorry"

USER WILL MANUALLY RUN (do not include in your changes):
pip install sentence-transformers langchain-huggingface

Start now:
- First read app/skills/rag_agent.py carefully
- Then implement all code changes with clear diffs
- Do NOT output any installation commands
- Keep Gemini for text generation if it's used (only remove embedding part)

This makes the project truly 100% free forever.