from sqlalchemy.orm import Session
from typing import Optional
from app.models import UserPreference, PersonalizedContent, BookChapter
from app.database import get_db
import logging

logger = logging.getLogger(__name__)

class PersonalizationService:
    """
    Service for handling content personalization based on user preferences
    """
    
    def __init__(self, db: Session):
        self.db = db
    
    def get_user_background(self, user_id: str) -> Optional[UserPreference]:
        """
        Retrieve user's background preferences
        """
        try:
            user_pref = self.db.query(UserPreference).filter(
                UserPreference.user_id == user_id
            ).first()
            return user_pref
        except Exception as e:
            logger.error(f"Error retrieving user preferences: {str(e)}")
            return None
    
    def adapt_content(self, content: str, user_id: str) -> str:
        """
        Adapt content based on user's background level
        """
        try:
            user_pref = self.get_user_background(user_id)
            if not user_pref:
                # Return original content if no preferences found
                return content
            
            background_level = user_pref.background_level.lower()
            
            # Apply different transformations based on background level
            if background_level == "novice":
                return self._adapt_for_novice(content)
            elif background_level == "intermediate":
                return self._adapt_for_intermediate(content)
            elif background_level == "advanced":
                return self._adapt_for_advanced(content)
            else:
                # Default to original content
                return content
        except Exception as e:
            logger.error(f"Error adapting content: {str(e)}")
            return content  # Return original content on error
    
    def _adapt_for_novice(self, content: str) -> str:
        """
        Adapt content for novice users by adding explanations and simpler language
        """
        # Add more detailed explanations, examples, and simpler language
        adapted_content = content
        
        # Replace complex terminology with explanations
        replacements = {
            "RAG": "Retrieval-Augmented Generation (RAG) - a technique that combines information retrieval with language models to improve accuracy",
            "kinematics": "kinematics - the study of motion without considering the forces that cause it",
            "dynamics": "dynamics - the study of motion and the forces that cause it",
            "PID controller": "PID controller (Proportional-Integral-Derivative controller) - a control loop mechanism that continuously calculates an error value as the difference between desired and measured values",
            "state estimation": "state estimation - the process of estimating the internal state of a system from sensor measurements",
            "sensor fusion": "sensor fusion - the process of combining data from multiple sensors to get more accurate and reliable information"
        }
        
        for term, explanation in replacements.items():
            adapted_content = adapted_content.replace(term, explanation)
        
        # Add encouraging notes for beginners
        adapted_content += "\n\n💡 **Beginner's Note**: Don't worry if some concepts seem complex at first. Take your time to understand each term and concept step by step."
        
        return adapted_content
    
    def _adapt_for_intermediate(self, content: str) -> str:
        """
        Adapt content for intermediate users by providing balanced explanations
        """
        # Return content with moderate level of detail
        adapted_content = content
        
        # Add some technical depth but keep explanations
        adapted_content += "\n\n🔧 **Implementation Tip**: As an intermediate learner, try implementing the concepts in small projects to reinforce your understanding."
        
        return adapted_content
    
    def _adapt_for_advanced(self, content: str) -> str:
        """
        Adapt content for advanced users by adding technical depth
        """
        # Return content with more technical depth and advanced concepts
        adapted_content = content
        
        # Add advanced insights
        adapted_content += "\n\n🔬 **Advanced Note**: For production implementations, consider additional factors like computational efficiency, security, and system robustness."
        
        return adapted_content
    
    def get_personalized_content(self, chapter_id: str, user_id: str) -> Optional[str]:
        """
        Retrieve existing personalized content or generate new one
        """
        try:
            # Try to find existing personalized content
            existing_content = self.db.query(PersonalizedContent).filter(
                PersonalizedContent.chapter_id == chapter_id,
                PersonalizedContent.user_id == user_id
            ).first()
            
            if existing_content:
                return existing_content.content_version
            
            # If no existing content, get user preferences to generate new content
            user_pref = self.get_user_background(user_id)
            if not user_pref:
                return None
            
            # Get original chapter content
            chapter = self.db.query(BookChapter).filter(
                BookChapter.id == chapter_id
            ).first()
            
            if not chapter:
                return None
            
            # Generate personalized content
            personalized_content = self.adapt_content(chapter.content, user_id)
            
            # Save personalized content for future use
            new_personalized = PersonalizedContent(
                chapter_id=chapter_id,
                user_preference_id=user_pref.id,
                content_version=personalized_content,
                personalization_level=user_pref.background_level
            )
            self.db.add(new_personalized)
            self.db.commit()
            
            return personalized_content
        except Exception as e:
            logger.error(f"Error retrieving personalized content: {str(e)}")
            return None

def create_personalization_service(db: Session) -> PersonalizationService:
    """
    Factory function to create a personalization service
    """
    return PersonalizationService(db)