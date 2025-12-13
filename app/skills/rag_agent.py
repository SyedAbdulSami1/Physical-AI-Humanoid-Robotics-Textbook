from qdrant_client import QdrantClient
from qdrant_client.http import models
import openai
from sqlalchemy.orm import Session
from typing import List, Dict, Any, Optional
import logging
import asyncio
from app.models import User, UserPreference

logger = logging.getLogger(__name__)

class RAGAgent:
    """
    Reusable RAG agent subagent for handling retrieval-augmented generation
    """
    
    def __init__(self, qdrant_client: QdrantClient, collection_name: str, openai_api_key: str, db: Session):
        self.qdrant_client = qdrant_client
        self.collection_name = collection_name
        self.openai_client = openai.OpenAI(api_key=openai_api_key)
        self.db = db
    
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
    
    async def generate_response(self, query: str, search_results: List[Any], user: Optional[User] = None) -> str:
        """
        Generate a response using retrieved content and LLM
        """
        try:
            # Format context from search results
            context_texts = []
            for result in search_results:
                payload = result.payload
                text = payload.get("text", "")
                context_texts.append(text)
            
            # Get user preferences for personalization
            user_context = ""
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
            
            # Combine context
            context = "\n\n".join(context_texts[:3])  # Use top 3 results
            full_context = f"{context}\n\n{user_context}" if user_context else context
            
            # Prepare the prompt for OpenAI
            system_prompt = (
                "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. "
                "Answer questions about humanoid robotics, physical AI, machine learning, control systems, "
                "sensor fusion, kinematics, dynamics, and related topics based on the provided context. "
                "Provide accurate, helpful, and technically sound explanations. "
                "If a question requires content not in the context, politely explain that you can only answer "
                "based on the textbook content provided."
            )
            
            user_prompt = f"Context:\n{full_context}\n\nQuestion: {query}\n\nAnswer:"
            
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
            
            return response.choices[0].message.content
        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            return "I apologize, but I encountered an error while processing your request. Please try again."
    
    async def get_selected_text_response(self, query: str, selected_text: str, user: Optional[User] = None) -> str:
        """
        Generate a response using only the selected text
        """
        try:
            # Get user preferences for personalization
            user_context = ""
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
            
            # Prepare the prompt for OpenAI
            system_prompt = (
                "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. "
                "Answer the following question based ONLY on the provided selected text. "
                "Do not use any external knowledge. "
                "Provide accurate, helpful, and technically sound explanations based only on the information provided. "
                "If the selected text does not contain information to answer the question, state that clearly."
            )
            
            full_context = f"{selected_text}\n\n{user_context}" if user_context else selected_text
            user_prompt = f"Selected Text:\n{full_context}\n\nQuestion: {query}\n\nAnswer:"
            
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
            
            return response.choices[0].message.content
        except Exception as e:
            logger.error(f"Error generating selected text response: {str(e)}")
            return "I apologize, but I encountered an error while processing your request. Please try again."