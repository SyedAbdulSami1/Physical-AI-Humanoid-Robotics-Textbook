# app/routers/ingest.py

import os
from dotenv import load_dotenv
from pathlib import Path
import re
from fastapi import APIRouter, Depends, HTTPException, status
from qdrant_client import QdrantClient, models as qdrant_models
from langchain_text_splitters import RecursiveCharacterTextSplitter
# Switched to local embeddings due to Google Gemini free tier embedding quota restrictions in 2026
from langchain_huggingface import HuggingFaceEmbeddings
from langchain_core.documents import Document
import markdown
from bs4 import BeautifulSoup

from app import schemas, models

# Load environment variables from .env file, overriding system variables
dotenv_path = Path(__file__).resolve().parent.parent / '.env'
load_dotenv(dotenv_path=dotenv_path, override=True)

router = APIRouter(
    prefix="/ingest",
    tags=["Ingestion"],
)

# --- Qdrant and Embeddings Setup ---

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = "textbook_content"

# Initialize clients
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
# Switched to local embeddings due to Google Gemini free tier embedding quota restrictions in 2026
embeddings = HuggingFaceEmbeddings(model_name="BAAI/bge-small-en-v1.5")

# Text splitter configuration
text_splitter = RecursiveCharacterTextSplitter(
    chunk_size=1000,
    chunk_overlap=200,
    length_function=len,
)

def md_to_text(md_content: str) -> str:
    """Convert Markdown content to plain text."""
    html = markdown.markdown(md_content)
    soup = BeautifulSoup(html, "html.parser")
    # Remove code blocks to avoid cluttering the text with code
    for code in soup.find_all("code"):
        code.decompose()
    return soup.get_text()

def get_documents_from_docs(docs_path: str = "docs") -> list[Document]:
    """
    Walks through the docs directory, reads all .md files, and returns a list of LangChain Documents.
    """
    documents = []
    for root, _, files in os.walk(docs_path):
        for file in files:
            if file.endswith(".md"):
                file_path = os.path.join(root, file)
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()
                    text_content = md_to_text(content)
                    
                    # Create a document with metadata
                    doc = Document(
                        page_content=text_content,
                        metadata={"source": file_path.replace("\\", "/")},
                    )
                    documents.append(doc)
    return documents



async def _run_ingestion_logic():
    """
    Contains the core logic for clearing, loading, chunking, and embedding content.
    """
    # 1. Delete and Recreate Collection
    try:
        qdrant_client.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=qdrant_models.VectorParams(
                size=384,  # bge-small-en-v1.5 dimension
                distance=qdrant_models.Distance.COSINE,
            ),
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to recreate Qdrant collection: {e}"
        )

    # 2. Load documents from the correct path
    # Construct the absolute path to the 'docs' directory, which is at the project root.
    # The current script is in /app/routers/, so we go up three levels.
    project_root = Path(__file__).resolve().parent.parent.parent
    docs_path = project_root / "docs"

    documents = get_documents_from_docs(docs_path=str(docs_path))
    if not documents:
        return {
            "message": f"Ingestion skipped: No documents found in the specified 'docs' directory: {docs_path}",
            "documents_found": 0,
            "chunks_created": 0,
            "vectors_upserted": 0,
        }

    # 3. Chunk documents
    chunked_docs = text_splitter.split_documents(documents)

    # 4. Generate embeddings and prepare for upsert
    points_to_upsert = []
    # Use a set to avoid duplicating work for identical content
    unique_contents = list({doc.page_content for doc in chunked_docs})
    
    try:
        # Generate embeddings for unique content only using the more efficient batch method
        embeddings_list = embeddings.embed_documents(unique_contents)
        embeddings_dict = dict(zip(unique_contents, embeddings_list))

        for i, doc in enumerate(chunked_docs):
            # Retrieve the pre-generated embedding
            embedding = embeddings_dict.get(doc.page_content)
            if embedding is None:
                # This should not happen if logic is correct, but as a fallback
                print(f"Warning: Could not find pre-generated embedding for content chunk {i}. Re-embedding individually.")
                embedding = embeddings.embed_query(doc.page_content)

            points_to_upsert.append(
                qdrant_models.PointStruct(
                    id=i, # Use a simple integer ID
                    vector=embedding,
                    payload={
                        "text": doc.page_content, # Include the text content in the payload
                        **doc.metadata
                    },
                )
            )
    except Exception as e:
        # This will catch API rate limit errors, etc.
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=f"Failed to generate embeddings. This is likely due to local model issues or resource constraints. Error: {e}"
        )

    # 5. Upsert to Qdrant in batches
    try:
        qdrant_client.upsert(
            collection_name=QDRANT_COLLECTION_NAME,
            points=points_to_upsert,
            wait=True,
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to upsert vectors into Qdrant: {e}"
        )

    return {
        "message": "Ingestion process completed successfully.",
        "documents_found": len(documents),
        "chunks_created": len(chunked_docs),
        "vectors_upserted": len(points_to_upsert),
    }


@router.post("/run", status_code=status.HTTP_200_OK)
async def run_ingestion():
    """
    Triggers the content ingestion process if the database is empty.
    
    This endpoint checks if the Qdrant collection already contains vectors.
    - If it's not empty, it returns a message indicating that the data is already ingested.
    - If it's empty, it automatically triggers the full ingestion process.
    
    This is designed to be a "safe" endpoint to call on application startup
    to ensure the database is populated without wastefully re-ingesting data.
    """
    try:
        collection_info = qdrant_client.get_collection(collection_name=QDRANT_COLLECTION_NAME)
        
        # Check if there are points in the collection
        if collection_info.points_count > 0:
            return {
                "message": "Ingestion has already been completed.",
                "points_in_collection": collection_info.points_count,
                "detail": "To force a re-ingestion of all documents, call the POST /ingest/force endpoint."
            }
            
    except Exception:
        # Collection likely does not exist, so we should proceed with ingestion.
        pass

    # If collection is empty or doesn't exist, run the ingestion logic.
    return await _run_ingestion_logic()


@router.post("/force", status_code=status.HTTP_200_OK)
async def force_ingestion():
    """
    Forcibly triggers a full re-ingestion of all content.
    
    This endpoint will:
    1. Delete the existing Qdrant collection.
    2. Re-create the collection.
    3. Re-read all documents from the `docs` directory.
    4. Re-generate embeddings for all content.
    5. Upsert all new vectors into the collection.
    
    Warning: This is an expensive operation and will consume a significant
    number of API calls. Only use this when you have made substantial
    updates to the source documents.
    """
    return await _run_ingestion_logic()

