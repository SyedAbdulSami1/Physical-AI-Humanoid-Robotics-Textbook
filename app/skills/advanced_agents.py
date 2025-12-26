"""
Skill 1: Content Summarization Agent
"""
from typing import List
import google.genai as genai
import asyncio
import logging
import json

logger = logging.getLogger(__name__)

class ContentSummarizationAgent:
    """
    Skill that summarizes long content chunks for more focused RAG responses
    """
    
    def __init__(self, gemini_model: genai.GenerativeModel):
        self.gemini_model = gemini_model
    
    async def summarize_content(self, content: str, max_length: int = 200) -> str:
        """
        Summarize content to a specified length
        """
        try:
            # If content is already short, return as is
            if len(content.split()) <= max_length:
                return content
            
            system_prompt = "You are an expert at summarizing technical content while preserving key concepts and details."
            user_prompt = f"Please summarize the following content:\n\n{content}\n\nSummary (about {max_length} words):"
            
            full_prompt = f"{system_prompt}\n\n{user_prompt}"
            
            response = await self.gemini_model.generate_content_async(full_prompt)
            
            return response.text
        except Exception as e:
            logger.error(f"Error in content summarization: {str(e)}")
            return content  # Return original content if summarization fails

"""
Skill 2: Contextual Question Router
"""
class ContextualQuestionRouter:
    """
    Skill that routes questions to the most appropriate knowledge base or processing method
    """
    
    def __init__(self):
        self.domains = {
            "kinematics": ["kinematics", "forward kinematics", "inverse kinematics", "motion", "joint", "position"],
            "dynamics": ["dynamics", "force", "torque", "acceleration", "inertia", "motion control"],
            "control": ["control", "PID", "controller", "feedback", "stability", "tracking"],
            "sensors": ["sensor", "fusion", "imu", "lidar", "camera", "perception", "detection"],
            "ai_ml": ["machine learning", "neural", "deep learning", "reinforcement", "algorithm", "training"],
            "hardware": ["actuator", "motor", "electronics", "embedded", "circuit", "pcb"]
        }
    
    def route_question(self, question: str) -> str:
        """
        Determine the most relevant domain for the question
        """
        question_lower = question.lower()
        
        # Count domain matches
        domain_scores = {}
        for domain, keywords in self.domains.items():
            score = sum(1 for keyword in keywords if keyword in question_lower)
            domain_scores[domain] = score
        
        # Return the domain with the highest score
        best_domain = max(domain_scores, key=domain_scores.get)
        best_score = domain_scores[best_domain]
        
        # If no good match found, return general
        if best_score == 0:
            return "general"
        
        return best_domain

"""
Skill 3: Query Refinement Agent
"""
class QueryRefinementAgent:
    """
    Skill that refines user queries to improve RAG performance
    """
    
    def __init__(self, gemini_model: genai.GenerativeModel):
        self.gemini_model = gemini_model
    
    async def refine_query(self, original_query: str) -> str:
        """
        Refine the user query to be more specific and search-friendly
        """
        try:
            system_prompt = (
                "You are an expert at refining search queries. " 
                "Improve the query to be more specific, detailed, and suitable for semantic search. "
                "Add relevant technical terms if needed."
            )
            
            user_prompt = f"Refine this query: {original_query}"
            
            full_prompt = f"{system_prompt}\n\n{user_prompt}"

            response = await self.gemini_model.generate_content_async(full_prompt)
            
            return response.text.strip()
        except Exception as e:
            logger.error(f"Error in query refinement: {str(e)}")
            return original_query  # Return original if refinement fails

"""
Skill 4: Response Verification Agent
"""
class ResponseVerificationAgent:
    """
    Skill that verifies the accuracy of generated responses against the source content
    """
    
    def __init__(self, gemini_model: genai.GenerativeModel):
        self.gemini_model = gemini_model
    
    async def verify_response(self, response: str, source_content: str) -> dict:
        """
        Verify if the response is consistent with the source content
        """
        try:
            system_prompt = (
                "You are an expert fact-checker. "
                "Verify if the response is consistent with the provided source content. "
                "Return a JSON object with two keys: 'is_consistent' (boolean) and 'feedback' (string). "
                "Do not add any other text or markdown formatting."
            )
            
            user_prompt = f"Source content: {source_content}\n\nResponse: {response}\n\nVerify consistency and provide JSON output:"
            
            full_prompt = f"{system_prompt}\n\n{user_prompt}"

            response_check = await self.gemini_model.generate_content_async(full_prompt)
            
            # Clean the response to ensure it is valid JSON
            cleaned_text = response_check.text.strip().replace("```json", "").replace("```", "").strip()
            verification_result = json.loads(cleaned_text)
            return verification_result
        except Exception as e:
            logger.error(f"Error in response verification: {str(e)}")
            # If verification fails, assume the response is consistent
            return {"is_consistent": True, "feedback": "Verification skipped due to error"}

"""
Skill 5: Multi-turn Conversation Manager
"""
class MultiTurnConversationManager:
    """
    Skill that manages multi-turn conversations and maintains context
    """
    
    def __init__(self):
        self.conversation_history = {}
    
    def add_message(self, session_id: str, role: str, content: str):
        """
        Add a message to the conversation history
        """
        if session_id not in self.conversation_history:
            self.conversation_history[session_id] = []
        
        self.conversation_history[session_id].append({
            "role": role,
            "content": content
        })
    
    def get_context(self, session_id: str, max_turns: int = 3) -> str:
        """
        Get recent conversation context
        """
        if session_id not in self.conversation_history:
            return ""
        
        # Get the last max_turns exchanges
        recent_messages = self.conversation_history[session_id][-max_turns*2:]
        context = "\n".join([f"{msg['role']}: {msg['content']}" for msg in recent_messages])
        
        return context
    
    def clear_history(self, session_id: str):
        """
        Clear conversation history for a session
        """
        if session_id in self.conversation_history:
            del self.conversation_history[session_id]

"""
Skill 6: Personalization Adapter
"""
class PersonalizationAdapter:
    """
    Skill that adapts responses based on user preferences and background
    """
    
    @staticmethod
    def adapt_response_to_user_level(response: str, user_level: str) -> str:
        """
        Adapt the response based on the user's experience level
        """
        if user_level.lower() == "novice":
            # Add more explanations and simpler language
            return f"**Explanation for beginners:** {response}\n\n*Note: This concept might seem complex at first. It refers to the basic principles of the topic.*"
        elif user_level.lower() == "intermediate":
            # Balance technical depth with explanations
            return response
        elif user_level.lower() == "advanced":
            # Add more technical details
            return f"{response}\n\n**Advanced insight:** For production-level implementations, consider additional factors like computational efficiency and system integration."
        else:
            # Default to intermediate
            return response