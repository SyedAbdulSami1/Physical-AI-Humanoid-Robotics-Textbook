# app/routers/chat.py

from fastapi import APIRouter, Depends, HTTPException, status
from app import schemas, models
from app.skills import rag_agent

router = APIRouter(
    prefix="/chat",
    tags=["Chatbot"],
)

@router.post("/", response_model=schemas.ChatResponse)
async def handle_chat_request(
    request: schemas.ChatRequest,
    current_user: models.User = Depends(schemas.get_current_active_user)
):
    """
    Handles a chat request by invoking the RAG agent.

    This endpoint is protected and requires user authentication. It takes a user's
    query and an optional `selected_text` field.

    - If `selected_text` is provided, the RAG agent will focus its search on
      content similar to the selected text.
    - If `selected_text` is not provided, the search is performed against the
      entire knowledge base.

    The RAG agent is responsible for:
    1. Embedding the user's query.
    2. Searching the Qdrant vector store for relevant documents.
    3. Constructing a prompt with the retrieved context.
    4. Calling the language model to generate a response.
    5. Returning the answer and the sources used.
    """
    if not current_user:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized",
        )

    try:
        # Delegate the entire logic to the RAG agent skill
        answer, sources = await rag_agent.process_query(
            query=request.query,
            selected_text=request.selected_text
        )
        
        return schemas.ChatResponse(answer=answer, sources=sources)

    except Exception as e:
        # Log the exception details in a real application
        print(f"Error in chat endpoint: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while processing your request.",
        )
