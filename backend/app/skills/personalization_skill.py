from typing import Dict, Any
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PersonalizationSkill:
    """
    A reusable skill for content personalization based on user profiles.
    """
    
    def __init__(self):
        self.name = "PersonalizationSkill"
        self.description = "A skill for personalizing content based on user profile"
    
    async def execute(self, content: str, profile: Dict[str, Any]) -> str:
        """
        Execute the personalization logic.
        
        Args:
            content (str): The original content to personalize
            profile (dict): User profile with experience levels
            
        Returns:
            str: Personalized content
        """
        try:
            logger.info(f"Executing {self.name} for profile: {profile}")
            
            # Extract profile information
            software_exp = profile.get('softwareExperience', '').lower()
            hardware_exp = profile.get('hardwareExperience', '').lower()
            
            # Apply personalization logic
            personalized_content = self._apply_personalization(content, software_exp, hardware_exp)
            
            logger.info(f"{self.name} executed successfully")
            return personalized_content
            
        except Exception as e:
            logger.error(f"Error in {self.name}: {str(e)}")
            raise
    
    def _apply_personalization(self, content: str, software_exp: str, hardware_exp: str) -> str:
        """
        Apply personalization logic to content.
        """
        # Add beginner-friendly explanations if user is a beginner
        if 'beginner' in [software_exp, hardware_exp]:
            content = self._add_beginner_explanations(content)
        
        # Modify complexity based on experience
        if software_exp == 'expert' and hardware_exp == 'expert':
            # For experts, add more advanced content
            content = self._add_advanced_explanations(content)
        elif 'beginner' in [software_exp, hardware_exp]:
            # For beginners, simplify explanations
            content = self._simplify_explanations(content)
        
        return content
    
    def _add_beginner_explanations(self, content: str) -> str:
        """
        Adds beginner-friendly explanations to content.
        """
        beginner_explanation = (
            "<div class='info-callout'>"
            "<strong>Beginner Note:</strong> For those new to this concept, "
            "this section covers fundamental principles. Take your time to "
            "understand each concept before moving forward."
            "</div>\n\n"
        )
        return f"{beginner_explanation}{content}"
    
    def _simplify_explanations(self, content: str) -> str:
        """
        Simplifies complex explanations for beginners.
        """
        # Replace complex terminology with simpler alternatives
        simplified_content = content.replace(
            "advanced concept", 
            "basic concept"
        ).replace(
            "complex implementation", 
            "simple approach"
        ).replace(
            "sophisticated algorithm", 
            "basic method"
        )
        
        return simplified_content
    
    def _add_advanced_explanations(self, content: str) -> str:
        """
        Adds advanced content for experienced users.
        """
        advanced_explanation = (
            "\n\n<div class='advanced-callout'>"
            "<strong>Advanced Insight:</strong> Experienced users might want to explore "
            "the mathematical foundations of this concept or consider how it applies "
            "to more complex robotics scenarios."
            "</div>"
        )
        
        return f"{content}{advanced_explanation}"

# Create a singleton instance
personalization_skill = PersonalizationSkill()