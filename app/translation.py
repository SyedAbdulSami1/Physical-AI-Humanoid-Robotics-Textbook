# app/translation.py

import os
import google.generativeai as genai
from fastapi import APIRouter, Depends, HTTPException

from app import schemas, models
from app.personalization import get_chapter_content  # Reuse the content reader

router = APIRouter(
    prefix="/translate",
    tags=["Translation"],
)

# --- LLM Setup ---
try:
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY environment variable not found.")
    genai.configure(api_key=GEMINI_API_KEY)
    llm = genai.GenerativeModel('gemini-1.5-pro')
except ValueError as e:
    llm = None
    print(f"Error initializing Gemini: {e}")


# --- Prompt Template ---
TRANSLATE_PROMPT_TEMPLATE = """
You are a highly skilled translator specializing in technical and academic content.
Your task is to translate the following English Markdown text into modern, formal Urdu.

INSTRUCTIONS:
1.  Translate the entire text accurately.
2.  Preserve the original Markdown formatting (headings, lists, bold, italics, code blocks, etc.).
3.  Do NOT translate text within code blocks (e.g., `def my_function():`).
4.  Ensure technical terms are translated correctly. If a standard Urdu equivalent does not exist, you may use the transliterated English term.
5.  The final output must be only the translated Markdown content, with no additional comments or explanations.

ENGLISH MARKDOWN CONTENT:
{chapter_content}

URDU MARKDOWN CONTENT:
"""

@router.post("/urdu", response_model=schemas.TranslationResponse)
async def translate_chapter_to_urdu(
    request: schemas.TranslationRequest,
    current_user: models.User = Depends(schemas.get_current_active_user)
):
    """
    Translates a chapter's content into Urdu for an authenticated user.
    """
    if not llm:
        raise HTTPException(status_code=500, detail="Translation service is not configured.")
        
    # 1. Get original chapter content
    try:
        chapter_content = get_chapter_content(request.chapter_id)
    except HTTPException as e:
        # Re-raise the exception if the chapter is not found or path is invalid
        raise e

    # 2. Manually format the prompt
    formatted_prompt = TRANSLATE_PROMPT_TEMPLATE.format(chapter_content=chapter_content)
    
    # 3. Invoke the Gemini LLM
    try:
        response = await llm.generate_content_async(formatted_prompt)
        translated_content = response.text
    except Exception as e:
        print(f"Error during Gemini API call for translation: {e}")
        raise HTTPException(status_code=500, detail="Failed to get a response from the translation service.")

    return schemas.TranslationResponse(translated_content=translated_content)
