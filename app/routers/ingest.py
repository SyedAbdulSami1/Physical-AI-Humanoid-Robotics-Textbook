# app/routers/ingest.py

import os
import re
from fastapi import APIRouter, Depends, HTTPException, status
from qdrant_client import QdrantClient, models as qdrant_models
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_google_genai import GoogleGenerativeAIEmbeddings
from langchain.docstore.document import Document
import markdown
from bs4 import BeautifulSoup

from app import schemas, models

router = APIRouter(
    prefix="/ingest",
    tags=["Ingestion"],
)

# --- Qdrant and Embeddings Setup ---

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_COLLECTION_NAME = "textbook_content"

# Initialize clients
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
embeddings = GoogleGenerativeAIEmbeddings(model="models/embedding-001", google_api_key=GEMINI_API_KEY)

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


@router.post("/run", status_code=status.HTTP_202_ACCEPTED)
async def run_ingestion(current_user: models.User = Depends(schemas.get_current_active_user)):
    """
    Protected endpoint to trigger the content ingestion process.
    This should be called only when the textbook content is updated.
    
    1. Deletes the existing Qdrant collection to ensure freshness.
    2. Creates a new collection.
    3. Reads all Markdown files from the `docs` directory.
    4. Chunks the documents.
    5. Generates embeddings for each chunk.
    6. Upserts the vectors into the Qdrant collection.
    """
    # For simplicity, only active users can trigger this. Add more granular permissions if needed.
    if not current_user.is_active:
        raise HTTPException(status_code=403, detail="Not authorized")

    # 1. Delete and Recreate Collection
    qdrant_client.recreate_collection(
        collection_name=QDRANT_COLLECTION_NAME,
        vectors_config=qdrant_models.VectorParams(
            size=768,  # Google Gemini embeddings dimension
            distance=qdrant_models.Distance.COSINE,
        ),
    )

    # 2. Load documents
    documents = get_documents_from_docs()
    if not documents:
        return {"message": "No documents found to ingest."}

    # 3. Chunk documents
    chunked_docs = text_splitter.split_documents(documents)

    # 4. Generate embeddings and prepare for upsert
    points_to_upsert = []
    for i, doc in enumerate(chunked_docs):
        embedding = embeddings.embed_query(doc.page_content)
        points_to_upsert.append(
            qdrant_models.PointStruct(
                id=i,
                vector=embedding,
                payload=doc.metadata,
            )
        )

    # 5. Upsert to Qdrant in batches
    qdrant_client.upsert(
        collection_name=QDRANT_COLLECTION_NAME,
        points=points_to_upsert,
        wait=True,
    )

    return {
        "message": "Ingestion process started successfully.",
        "documents_found": len(documents),
        "chunks_created": len(chunked_docs),
        "vectors_upserted": len(points_to_upsert),
    }
