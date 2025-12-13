# app/personalization.py

import os
from fastapi import APIRouter, Depends, HTTPException
from langchain_openai import ChatOpenAI
from langchain.prompts import ChatPromptTemplate

from app import schemas, models

router = APIRouter(
    prefix="/personalize",
    tags=["Personalization"],
)

# --- LLM Setup ---
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
llm = ChatOpenAI(model="gpt-4-turbo", temperature=0.1, api_key=OPENAI_API_KEY)

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

personalize_prompt = ChatPromptTemplate.from_template(PERSONALIZE_PROMPT_TEMPLATE)

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
    if not current_user.profile:
        raise HTTPException(status_code=404, detail="User profile not found.")

    # 1. Get original chapter content
    chapter_content = get_chapter_content(request.chapter_id)

    # 2. Construct the chain and invoke the LLM
    chain = personalize_prompt | llm
    
    response = await chain.ainvoke({
        "role": current_user.profile.role.value if current_user.profile.role else 'Not specified',
        "expertise": current_user.profile.technical_expertise.value if current_user.profile.technical_expertise else 'Not specified',
        "interest": current_user.profile.primary_interest or 'Not specified',
        "style": current_user.profile.preferred_explanation_style.value if current_user.profile.preferred_explanation_style else 'beginner',
        "chapter_content": chapter_content,
    })

    personalized_content = response.content

    return schemas.PersonalizeResponse(personalized_content=personalized_content)
