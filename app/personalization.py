# app/personalization.py

import os
import google.generativeai as genai
from fastapi import APIRouter, Depends, HTTPException

from app import schemas, models

router = APIRouter(
    prefix="/personalize",
    tags=["Personalization"],
)

# --- LLM Setup ---
# Use the native Google aPI as requested
try:
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY environment variable not found.")
    genai.configure(api_key=GEMINI_API_KEY)
    llm = genai.GenerativeModel('gemini-1.5-pro')
except ValueError as e:
    # Handle the case where the API key is missing
    llm = None
    print(f"Error initializing Gemini: {e}")


# --- Prompt Template ---
PERSONALIZE_PROMPT_TEMPLATE = """
You are an expert educator adapting a textbook chapter for a specific user.
Your task is to rewrite the given chapter content to match the user's background and learning style.

USER PROFILE:
- Role: {role}
- Technical Expertise: {expertise}
- Primary Interest: {interest}
- Preferred Style: {style}

INSTRUCTIONS:
1.  Read the original chapter content carefully.
2.  Subtly rewrite the content to better suit the user.
    - For a 'BEGINNER', simplify complex terms, add more foundational explanations, and use analogies.
    - For an 'ADVANCED' or 'EXPERT', use more technical jargon, focus on deeper implications, and perhaps add more complex code examples or theoretical notes.
    - Try to connect the content to the user's 'Primary Interest' where it feels natural.
3.  Do NOT add or remove major topics. The core structure must remain the same.
4.  The output MUST be valid Markdown. Preserve the original Markdown formatting (headings, lists, code blocks, etc.).
5.  Do NOT add any commentary like "Here is the personalized version". Output only the rewritten Markdown.

ORIGINAL CHAPTER CONTENT:
{chapter_content}

REWRITTEN AND PERSONALIZED MARKDOWN:
"""

def get_chapter_content(chapter_id: str) -> str:
    """
    Reads the content of a chapter's Markdown file from the `docs` directory.
    `chapter_id` should be the relative path of the file, e.g., 'module-1/ros2-nodes-topics-services.md'.
    """
    # Basic security to prevent directory traversal
    safe_path = os.path.normpath(os.path.join("docs", chapter_id))
    if not safe_path.startswith("docs"):
        raise HTTPException(status_code=400, detail="Invalid chapter ID")
    
    if not os.path.exists(safe_path) or not safe_path.endswith(".md"):
        raise HTTPException(status_code=404, detail="Chapter not found")
        
    with open(safe_path, "r", encoding="utf-8") as f:
        return f.read()

@router.post("/", response_model=schemas.PersonalizeResponse)
async def personalize_chapter_content(
    request: schemas.PersonalizeRequest,
    current_user: models.User = Depends(schemas.get_current_active_user)
):
    """
    Personalizes a chapter's content based on the authenticated user's profile.
    """
    if not llm:
        raise HTTPException(status_code=500, detail="Personalization service is not configured.")

    if not current_user.profile:
        raise HTTPException(status_code=404, detail="User profile not found.")

    # 1. Get original chapter content
    chapter_content = get_chapter_content(request.chapter_id)

    # 2. Manually format the prompt
    formatted_prompt = PERSONALIZE_PROMPT_TEMPLATE.format(
        role=current_user.profile.role.value if current_user.profile.role else 'Not specified',
        expertise=current_user.profile.technical_expertise.value if current_user.profile.technical_expertise else 'Not specified',
        interest=current_user.profile.primary_interest or 'Not specified',
        style=current_user.profile.preferred_explanation_style.value if current_user.profile.preferred_explanation_style else 'beginner',
        chapter_content=chapter_content,
    )
    
    # 3. Invoke the Gemini LLM
    try:
        response = await llm.generate_content_async(formatted_prompt)
        personalized_content = response.text
    except Exception as e:
        print(f"Error during Gemini API call: {e}")
        raise HTTPException(status_code=500, detail="Failed to get a response from the personalization service.")


    return schemas.PersonalizeResponse(personalized_content=personalized_content)
