import asyncio
from typing import Dict, Any
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def personalize_content_based_on_profile(content: str, profile: Dict[str, Any]) -> str:
    """
    Personalizes content based on user profile.
    This function adapts the content based on the user's software and hardware experience.
    """
    try:
        # Log the personalization request
        logger.info(f"Personalizing content for profile: {profile}")
        
        # Extract profile information
        software_exp = profile.get('softwareExperience', '').lower()
        hardware_exp = profile.get('hardwareExperience', '').lower()
        
        # Personalize the content based on experience level
        personalized_content = content
        
        # Add beginner-friendly explanations if user is a beginner
        if 'beginner' in [software_exp, hardware_exp]:
            # Add explanations for complex terms and concepts
            personalized_content = add_beginner_explanations(content)
        
        # Modify complexity based on experience
        if software_exp == 'expert' and hardware_exp == 'expert':
            # For experts, add more advanced content
            personalized_content = add_advanced_explanations(personalized_content)
        elif 'beginner' in [software_exp, hardware_exp]:
            # For beginners, simplify explanations
            personalized_content = simplify_explanations(personalized_content)
        
        # Log success
        logger.info("Content personalized successfully")
        return personalized_content
        
    except Exception as e:
        logger.error(f"Personalization error: {str(e)}")
        raise Exception(f"Personalization service error: {str(e)}")

def add_beginner_explanations(content: str) -> str:
    """
    Adds beginner-friendly explanations to content.
    """
    beginner_explanations = [
        "<div class='info-callout'><strong>Beginner Note:</strong> For those new to this concept, think of it as...</div>",
        "<div class='info-callout'><strong>Beginner Tip:</strong> If you're not familiar with this term, it means...</div>",
    ]
    
    # In a real implementation, this would intelligently insert explanations
    # at appropriate places in the content based on keywords or sections.
    # For this example, we'll just add a general beginner note.
    beginner_note = f"{beginner_explanations[0]}\n\n{content}"
    return beginner_note

def simplify_explanations(content: str) -> str:
    """
    Simplifies complex explanations in content for beginners.
    """
    # In a real implementation, this would replace complex terminology
    # with simpler alternatives and add more detailed explanations
    # for difficult concepts.
    simplified_content = content.replace(
        "advanced concept", 
        "basic concept"
    ).replace(
        "complex implementation", 
        "simple approach"
    )
    
    return simplified_content

def add_advanced_explanations(content: str) -> str:
    """
    Adds advanced content for experienced users.
    """
    # In a real implementation, this would add more sophisticated 
    # explanations and deeper technical details
    advanced_content = f"{content}\n\n<div class='advanced-callout'><strong>Advanced Insight:</strong> For more experienced users, consider the implications of...</div>"
    
    return advanced_content