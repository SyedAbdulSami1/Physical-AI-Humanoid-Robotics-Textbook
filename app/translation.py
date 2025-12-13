# app/translation.py

import os
from fastapi import APIRouter, Depends, HTTPException
from langchain_openai import ChatOpenAI
from langchain.prompts import ChatPromptTemplate

from app import schemas, models
from app.personalization import get_chapter_content  # Reuse the content reader

router = APIRouter(
    prefix="/translate",
    tags=["Translation"],
)

# --- LLM Setup ---
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
llm = ChatOpenAI(model="gpt-4-turbo", temperature=0.3, api_key=OPENAI_API_KEY)

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

translate_prompt = ChatPromptTemplate.from_template(TRANSLATE_PROMPT_TEMPLATE)

@router.post("/urdu", response_model=schemas.TranslationResponse)
async def translate_chapter_to_urdu(
    request: schemas.TranslationRequest,
    current_user: models.User = Depends(schemas.get_current_active_user)
):
    """
    Translates a chapter's content into Urdu for an authenticated user.
    """
    # 1. Get original chapter content
    try:
        chapter_content = get_chapter_content(request.chapter_id)
    except HTTPException as e:
        # Re-raise the exception if the chapter is not found or path is invalid
        raise e

    # 2. Construct the chain and invoke the LLM
    chain = translate_prompt | llm
    
    response = await chain.ainvoke({
        "chapter_content": chapter_content,
    })

    translated_content = response.content

    return schemas.TranslationResponse(translated_content=translated_content)
