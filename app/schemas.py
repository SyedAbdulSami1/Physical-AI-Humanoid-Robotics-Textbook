# app/schemas.py

from pydantic import BaseModel, EmailStr, Field
from typing import Optional, List
from datetime import datetime




# --- Chat Schemas ---

class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    sources: List[str]


