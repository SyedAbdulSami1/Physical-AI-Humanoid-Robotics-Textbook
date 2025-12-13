"""
Comprehensive RAG Agent that integrates all subagents for enhanced functionality
"""
from qdrant_client import QdrantClient
from qdrant_client.http import models
import openai
from sqlalchemy.orm import Session
from typing import List, Dict, Any, Optional
import logging
import asyncio
from app.models import User, UserPreference
from app.skills.advanced_agents import (
    ContentSummarizationAgent,
    ContextualQuestionRouter,
    QueryRefinementAgent,
    ResponseVerificationAgent,
    MultiTurnConversationManager,
    PersonalizationAdapter
)

logger = logging.getLogger(__name__)

class ComprehensiveRAGAgent:
    """
    Main RAG agent that integrates multiple subagents for enhanced functionality
    """
    
    def __init__(self, qdrant_client: QdrantClient, collection_name: str, openai_api_key: str, db: Session):
        self.qdrant_client = qdrant_client
        self.collection_name = collection_name
        self.openai_client = openai.OpenAI(api_key=openai_api_key)
        self.db = db
        
        # Initialize all subagents
        self.summarization_agent = ContentSummarizationAgent(self.openai_client)
        self.question_router = ContextualQuestionRouter()
        self.query_refiner = QueryRefinementAgent(self.openai_client)
        self.response_verifier = ResponseVerificationAgent(self.openai_client)
        self.conversation_manager = MultiTurnConversationManager()
        self.personalization_adapter = PersonalizationAdapter()
    
    def search_content(self, query: str, top_k: int = 5, chapter_slug: Optional[str] = None) -> List[Any]:
        """
        Search the vector database for relevant content
        """
        try:
            # Embed the query using OpenAI's embedding model
            response = self.openai_client.embeddings.create(
                input=query,
                model="text-embedding-ada-002"
            )
            query_vector = response.data[0].embedding
            
            # Prepare search filters
            search_filter = None
            if chapter_slug:
                search_filter = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="chapter_slug",
                            match=models.MatchValue(value=chapter_slug)
                        )
                    ]
                )
            
            # Perform similarity search
            results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                query_filter=search_filter,
                limit=top_k
            )
            
            return results
        except Exception as e:
            logger.error(f"Error during content search: {str(e)}")
            return []
    
    async def generate_response(self, query: str, search_results: List[Any], user: Optional[User] = None, session_id: Optional[str] = None) -> str:
        """
        Generate a response using retrieved content and LLM with all subagent enhancements
        """
        try:
            # Get conversation context if available
            conversation_context = ""
            if session_id:
                conversation_context = self.conversation_manager.get_context(session_id)
            
            # Get user preferences for personalization
            user_context = ""
            user_level = "intermediate"  # Default level
            if user:
                user_prefs = self.db.query(UserPreference).filter(
                    UserPreference.user_id == user.id
                ).first()
                
                if user_prefs:
                    user_context = f"\n\nUser Background: {user_prefs.background_level} level in robotics/AI. "
                    if user_prefs.programming_language:
                        user_context += f"Prefers {user_prefs.programming_language}. "
                    if user_prefs.hardware_interest:
                        user_context += f"Interested in {user_prefs.hardware_interest}. "
                    user_level = user_prefs.background_level
            
            # Refine the query for better results
            refined_query = await self.query_refiner.refine_query(query)
            
            # Route the question to appropriate domain
            domain = self.question_router.route_question(refined_query)
            
            # Format context from search results
            context_texts = []
            for result in search_results:
                payload = result.payload
                text = payload.get("text", "")
                
                # Summarize long content chunks
                summarized_text = await self.summarization_agent.summarize_content(text)
                context_texts.append(summarized_text)
            
            # Combine context
            context = "\n\n".join(context_texts[:3])  # Use top 3 results
            full_context = f"{context}\n\n{user_context}"
            
            if conversation_context:
                full_context = f"Previous conversation:\n{conversation_context}\n\nCurrent context:\n{full_context}"
            
            # Prepare the prompt for OpenAI
            system_prompt = (
                f"You are an expert assistant for the Physical AI & Humanoid Robotics textbook. "
                f"Answer questions about humanoid robotics, physical AI, machine learning, control systems, "
                f"sensor fusion, kinematics, dynamics, and related topics in the {domain} domain based on the provided context. "
                f"Provide accurate, helpful, and technically sound explanations. "
                f"If a question requires content not in the context, politely explain that you can only answer "
                f"based on the textbook content provided."
            )
            
            user_prompt = f"Context:\n{full_context}\n\nQuestion: {refined_query}\n\nAnswer:"
            
            # Call OpenAI API asynchronously
            response = await self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=1000,
                temperature=0.7
            )
            
            generated_response = response.choices[0].message.content
            
            # Verify the response against source content
            verification = await self.response_verifier.verify_response(generated_response, context)
            if not verification.get('is_consistent', True):
                logger.warning(f"Response may be inconsistent with source: {verification.get('feedback', 'No feedback')}")
            
            # Adapt response to user level
            adapted_response = self.personalization_adapter.adapt_response_to_user_level(
                generated_response, 
                user_level
            )
            
            # Add to conversation history if session is provided
            if session_id:
                self.conversation_manager.add_message(session_id, "user", query)
                self.conversation_manager.add_message(session_id, "assistant", adapted_response)
            
            return adapted_response
        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            return "I apologize, but I encountered an error while processing your request. Please try again."
    
    async def get_selected_text_response(self, query: str, selected_text: str, user: Optional[User] = None, session_id: Optional[str] = None) -> str:
        """
        Generate a response using only the selected text with subagent enhancements
        """
        try:
            # Get conversation context if available
            conversation_context = ""
            if session_id:
                conversation_context = self.conversation_manager.get_context(session_id)
            
            # Get user preferences for personalization
            user_context = ""
            user_level = "intermediate"  # Default level
            if user:
                user_prefs = self.db.query(UserPreference).filter(
                    UserPreference.user_id == user.id
                ).first()
                
                if user_prefs:
                    user_context = f"\n\nUser Background: {user_prefs.background_level} level in robotics/AI. "
                    user_level = user_prefs.background_level
            
            # Refine the query for better results
            refined_query = await self.query_refiner.refine_query(query)
            
            # Summarize selected text if it's long
            summarized_text = await self.summarization_agent.summarize_content(selected_text)
            
            # Prepare the prompt for OpenAI
            system_prompt = (
                "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. "
                "Answer the following question based ONLY on the provided selected text. "
                "Do not use any external knowledge. "
                "Provide accurate, helpful, and technically sound explanations based only on the information provided. "
                "If the selected text does not contain information to answer the question, state that clearly."
            )
            
            full_context = f"{summarized_text}\n\n{user_context}"
            if conversation_context:
                full_context = f"Previous conversation:\n{conversation_context}\n\nCurrent selected text:\n{full_context}"
                
            user_prompt = f"Selected Text:\n{full_context}\n\nQuestion: {refined_query}\n\nAnswer:"
            
            # Call OpenAI API asynchronously
            response = await self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=1000,
                temperature=0.7
            )
            
            generated_response = response.choices[0].message.content
            
            # Verify the response against source content
            verification = await self.response_verifier.verify_response(generated_response, summarized_text)
            if not verification.get('is_consistent', True):
                logger.warning(f"Response may be inconsistent with source: {verification.get('feedback', 'No feedback')}")
            
            # Adapt response to user level
            adapted_response = self.personalization_adapter.adapt_response_to_user_level(
                generated_response, 
                user_level
            )
            
            # Add to conversation history if session is provided
            if session_id:
                self.conversation_manager.add_message(session_id, "user", query)
                self.conversation_manager.add_message(session_id, "assistant", adapted_response)
            
            return adapted_response
        except Exception as e:
            logger.error(f"Error generating selected text response: {str(e)}")
            return "I apologize, but I encountered an error while processing your request. Please try again."