from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel
import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
import openai
from typing import List
from app.database import get_db
from app.models import BookChapter
import logging

router = APIRouter()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize Qdrant client
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "textbook_content")

qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Initialize OpenAI client
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
openai.api_key = OPENAI_API_KEY

class IngestChapterRequest(BaseModel):
    title: str
    slug: str
    content: str

class BulkIngestRequest(BaseModel):
    chapters: List[IngestChapterRequest]

def create_collection():
    """
    Create Qdrant collection if it doesn't exist
    """
    try:
        # Check if collection exists
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]
        
        if QDRANT_COLLECTION_NAME not in collection_names:
            # Create collection with vector configuration
            qdrant_client.create_collection(
                collection_name=QDRANT_COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE)  # OpenAI embedding size
            )
            
            # Create payload index for chapter_slug
            qdrant_client.create_payload_index(
                collection_name=QDRANT_COLLECTION_NAME,
                field_name="chapter_slug",
                field_schema=models.PayloadSchemaType.KEYWORD
            )
    except Exception as e:
        logger.error(f"Error creating collection: {str(e)}")
        raise

def chunk_text(text: str, chunk_size: int = 1000) -> List[str]:
    """
    Split text into chunks of specified size
    """
    chunks = []
    for i in range(0, len(text), chunk_size):
        chunk = text[i:i+chunk_size]
        chunks.append(chunk)
    return chunks

def embed_text(text: str) -> List[float]:
    """
    Generate embedding for text using OpenAI
    """
    try:
        response = openai.embeddings.create(
            input=text,
            model="text-embedding-ada-002"
        )
        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Error generating embedding: {str(e)}")
        raise

@router.post("/ingest-chapter")
def ingest_chapter(request: IngestChapterRequest, db: Session = Depends(get_db)):
    """
    Ingest a single chapter into the vector database
    """
    try:
        # Check if chapter already exists
        existing_chapter = db.query(BookChapter).filter(BookChapter.slug == request.slug).first()
        if existing_chapter:
            raise HTTPException(status_code=400, detail="Chapter with this slug already exists")
        
        # Create collection if it doesn't exist
        create_collection()
        
        # Chunk the content
        text_chunks = chunk_text(request.content)
        
        # Prepare points for Qdrant
        points = []
        for i, chunk in enumerate(text_chunks):
            # Generate embedding for the chunk
            vector = embed_text(chunk)
            
            # Create a point for Qdrant
            point = models.PointStruct(
                id=f"{request.slug}_{i}",
                vector=vector,
                payload={
                    "text": chunk,
                    "chapter_title": request.title,
                    "chapter_slug": request.slug,
                    "chunk_index": i
                }
            )
            points.append(point)
        
        # Upload points to Qdrant
        qdrant_client.upsert(
            collection_name=QDRANT_COLLECTION_NAME,
            points=points
        )
        
        # Save chapter metadata to PostgreSQL
        chapter = BookChapter(
            title=request.title,
            slug=request.slug,
            content=request.content
        )
        db.add(chapter)
        db.commit()
        db.refresh(chapter)
        
        return {
            "message": f"Successfully ingested chapter '{request.title}' with {len(text_chunks)} chunks",
            "chapter_id": chapter.id,
            "chunks_ingested": len(text_chunks)
        }
    except Exception as e:
        logger.error(f"Ingestion error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to ingest chapter: {str(e)}")

@router.post("/bulk-ingest")
def bulk_ingest(request: BulkIngestRequest, db: Session = Depends(get_db)):
    """
    Ingest multiple chapters at once
    """
    try:
        create_collection()
        results = []
        
        for chapter_request in request.chapters:
            # Check if chapter already exists
            existing_chapter = db.query(BookChapter).filter(
                BookChapter.slug == chapter_request.slug
            ).first()
            
            if existing_chapter:
                results.append({
                    "title": chapter_request.title,
                    "status": "skipped",
                    "message": "Chapter already exists"
                })
                continue
            
            # Chunk the content
            text_chunks = chunk_text(chapter_request.content)
            
            # Prepare points for Qdrant
            points = []
            for i, chunk in enumerate(text_chunks):
                # Generate embedding for the chunk
                vector = embed_text(chunk)
                
                # Create a point for Qdrant
                point = models.PointStruct(
                    id=f"{chapter_request.slug}_{len(points)}",
                    vector=vector,
                    payload={
                        "text": chunk,
                        "chapter_title": chapter_request.title,
                        "chapter_slug": chapter_request.slug,
                        "chunk_index": i
                    }
                )
                points.append(point)
            
            # Upload points to Qdrant
            qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION_NAME,
                points=points
            )
            
            # Save chapter metadata to PostgreSQL
            chapter = BookChapter(
                title=chapter_request.title,
                slug=chapter_request.slug,
                content=chapter_request.content
            )
            db.add(chapter)
            
            results.append({
                "title": chapter_request.title,
                "status": "success",
                "chunks_ingested": len(text_chunks)
            })
        
        # Commit all chapters to the database
        db.commit()
        
        return {
            "message": "Bulk ingestion completed",
            "results": results
        }
    except Exception as e:
        logger.error(f"Bulk ingestion error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to bulk ingest chapters: {str(e)}")

@router.post("/rebuild-index")
def rebuild_index(db: Session = Depends(get_db)):
    """
    Rebuild the entire vector index from database content
    """
    try:
        # Delete existing collection
        try:
            qdrant_client.delete_collection(QDRANT_COLLECTION_NAME)
        except:
            pass  # Collection might not exist yet
        
        # Create new collection
        create_collection()
        
        # Get all chapters from database
        chapters = db.query(BookChapter).all()
        
        total_chunks = 0
        for chapter in chapters:
            # Chunk the content
            text_chunks = chunk_text(chapter.content)
            
            # Prepare points for Qdrant
            points = []
            for i, chunk in enumerate(text_chunks):
                # Generate embedding for the chunk
                vector = embed_text(chunk)
                
                # Create a point for Qdrant
                point = models.PointStruct(
                    id=f"{chapter.slug}_{len(points)}",
                    vector=vector,
                    payload={
                        "text": chunk,
                        "chapter_title": chapter.title,
                        "chapter_slug": chapter.slug,
                        "chunk_index": i
                    }
                )
                points.append(point)
            
            # Upload points to Qdrant
            qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION_NAME,
                points=points
            )
            
            total_chunks += len(text_chunks)
        
        return {
            "message": f"Successfully rebuilt index with {len(chapters)} chapters and {total_chunks} chunks",
            "chapters_processed": len(chapters),
            "chunks_processed": total_chunks
        }
    except Exception as e:
        logger.error(f"Index rebuild error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to rebuild index: {str(e)}")

@router.get("/status")
def ingestion_status():
    """
    Get the status of the ingestion pipeline
    """
    try:
        # Get collection info
        collection_info = qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
        
        return {
            "status": "ready",
            "collection_name": QDRANT_COLLECTION_NAME,
            "vectors_count": collection_info.points_count,
            "indexed_chapters": "Count requires manual tracking"
        }
    except Exception as e:
        logger.error(f"Status check error: {str(e)}")
        return {
            "status": "error",
            "message": str(e)
        }