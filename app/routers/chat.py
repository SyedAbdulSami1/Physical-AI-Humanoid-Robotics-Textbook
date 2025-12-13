from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional
from pydantic import BaseModel
import os
import asyncio
from qdrant_client import QdrantClient
from qdrant_client.http import models
import openai
from app.database import get_db
from app.auth import get_current_user
from app.models import User, ChatHistory, BookChapter
from app.skills import ComprehensiveRAGAgent  # Import the comprehensive agent
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

class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None  # For selected-text-only mode
    chapter_slug: Optional[str] = None  # For contextual responses
    user_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: list = []
    selected_text_used: bool = False

@router.post("/message", response_model=ChatResponse)
async def chat_message(
    request: ChatRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Main chat endpoint that handles both full-book and selected-text-only modes
    """
    try:
        # Create Comprehensive RAG agent instance
        rag_agent = ComprehensiveRAGAgent(
            qdrant_client=qdrant_client,
            collection_name=QDRANT_COLLECTION_NAME,
            openai_api_key=OPENAI_API_KEY,
            db=db
        )

        # Determine if we're in selected-text-only mode
        if request.selected_text:
            # Selected text only mode - only use the provided selected text
            response = await rag_agent.get_selected_text_response(
                query=request.message,
                selected_text=request.selected_text,
                user=current_user
            )

            # Save chat history
            chat_history = ChatHistory(
                user_input=request.message,
                bot_response=response,
                selected_text=request.selected_text,
                chapter_context=request.chapter_slug
            )
            db.add(chat_history)
            db.commit()

            return ChatResponse(
                response=response,
                sources=[],
                selected_text_used=True
            )
        else:
            # Full book content mode - search through all textbook content
            search_results = rag_agent.search_content(query=request.message, top_k=5, chapter_slug=request.chapter_slug)
            response = await rag_agent.generate_response(
                query=request.message,
                search_results=search_results,
                user=current_user
            )

            # Extract source documents as sources
            sources = [result.payload.get('source', '') for result in search_results]

            # Save chat history
            chat_history = ChatHistory(
                user_input=request.message,
                bot_response=response,
                chapter_context=request.chapter_slug
            )
            db.add(chat_history)
            db.commit()

            return ChatResponse(
                response=response,
                sources=sources,
                selected_text_used=False
            )
    except Exception as e:
        logger.error(f"Chat error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/history")
async def get_chat_history(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Retrieve chat history for the authenticated user
    """
    try:
        history = db.query(ChatHistory).filter(
            ChatHistory.user_id == current_user.id
        ).order_by(ChatHistory.created_at.desc()).all()
        return {"history": history}
    except Exception as e:
        logger.error(f"Error retrieving chat history: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/clear-history")
async def clear_chat_history(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Clear chat history for the authenticated user
    """
    try:
        db.query(ChatHistory).filter(ChatHistory.user_id == current_user.id).delete()
        db.commit()
        return {"message": "Chat history cleared successfully"}
    except Exception as e:
        logger.error(f"Error clearing chat history: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")