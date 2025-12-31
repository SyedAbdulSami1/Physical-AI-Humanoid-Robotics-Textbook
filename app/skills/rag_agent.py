# app/skills/rag_agent.py

import os
from typing import List, Tuple
from qdrant_client import QdrantClient
from langchain_google_genai import GoogleGenerativeAIEmbeddings, ChatGoogleGenerativeAI
from langchain_core.prompts import ChatPromptTemplate
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RAGAgent:
    def __init__(self):
        # --- Client and Model Setup ---
        # --- [FORENSIC DEBUG] ---
        # Enhanced debugging to find corrupted environment variables.
        import os
        from dotenv import load_dotenv
        from pathlib import Path

        print("--- [RAG AGENT FORENSIC DEBUG] ---")
        # Ensure the .env is loaded from the correct location
        dotenv_path = Path(__file__).resolve().parent.parent / '.env'
        print(f"  - Loading .env from: {dotenv_path}")
        print(f"  - .env file exists: {dotenv_path.exists()}")

        # Load the .env file explicitly
        load_dotenv(dotenv_path=dotenv_path, override=True)  # Override system variables

        key = os.getenv("GEMINI_API_KEY")
        qdrant_url = os.getenv("QDRANT_URL") # FIX: Define qdrant_url

        if key:
            print(f"  - Gemini Key Loaded: Yes")
            # repr() will show hidden characters like '\\n', '\\r', quotes, etc.
            print(f"  - repr(key): {repr(key)}")
            print(f"  - Exact length: {len(key)}")
            print(f"  - First 10 chars: {key[:10]}")
            print(f"  - Last 5 chars: {key[-5:]}")
            print(f"  - Starts with 'AIzaSy': {key.startswith('AIzaSy') if key else False}")
        else:
            print(f"  - Gemini Key Loaded: No (is None)")

        if qdrant_url:
            print(f"  - Qdrant URL Loaded: Yes")
        else:
            print(f"  - Qdrant URL Loaded: No (is None)")

        print("--- [END FORENSIC DEBUG] ---")

        self.GEMINI_API_KEY = key
        self.QDRANT_URL = qdrant_url
        self.QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
        self.QDRANT_COLLECTION_NAME = "textbook_content"

        if not all([self.QDRANT_URL, self.QDRANT_API_KEY, self.GEMINI_API_KEY]):
            raise ValueError("Missing required environment variables for RAG agent. Check .env file and server start location.")

        # Initialize clients and models
        try:
            self.qdrant_client = QdrantClient(url=self.QDRANT_URL, api_key=self.QDRANT_API_KEY)
            self.embeddings = GoogleGenerativeAIEmbeddings(
                model="models/embedding-001",
                google_api_key=self.GEMINI_API_KEY
            )
            self.llm = ChatGoogleGenerativeAI(
                model="gemini-1.5-flash",
                temperature=0.1,
                google_api_key=self.GEMINI_API_KEY
            )
        except Exception as e:
            logger.error(f"Failed to initialize RAG agent components: {e}")
            raise

        # --- Prompt Template ---
        self.RAG_PROMPT_TEMPLATE = """
You are a helpful assistant for the "Physical AI & Humanoid Robotics" textbook.
Your task is to answer the user's question based *only* on the provided context.
If the context does not contain the answer, state that you cannot answer the question with the information provided.
Be concise and clear. Cite the sources you used by listing their file paths at the end of your answer under a "Sources:" heading.

CONTEXT:
{context}

QUESTION:
{question}

ANSWER:
"""
        self.rag_prompt = ChatPromptTemplate.from_template(self.RAG_PROMPT_TEMPLATE)

    async def _search_qdrant(self, query: str, top_k: int = 5) -> List[dict]:
        """
        Searches the Qdrant collection for the most relevant document chunks.
        """
        try:
            # First, check if the collection exists.
            try:
                self.qdrant_client.get_collection(collection_name=self.QDRANT_COLLECTION_NAME)
            except Exception:
                logger.warning(
                    f"Qdrant collection '{self.QDRANT_COLLECTION_NAME}' not found. "
                    "Returning empty search results. Please run the ingestion process."
                )
                return []

            vector = self.embeddings.embed_query(query)
            search_results = self.qdrant_client.search(
                collection_name=self.QDRANT_COLLECTION_NAME,
                query_vector=vector,
                limit=top_k,
                with_payload=True,
            )
            return [result.model_dump() for result in search_results]
        except Exception as e:
            logger.error(f"Error searching Qdrant: {e}")
            # Re-raise the exception to be handled by the endpoint, which will return a 500 error.
            # This is important for visibility into API key errors, connection issues, etc.
            raise

    async def generate_response(self, query: str, selected_text: str = None, search_results: List[dict] = None) -> Tuple[str, List[str]]:
        """
        Processes a user query through the RAG pipeline.
        """
        if search_results is None:
            search_query = f"{selected_text}\n\n{query}" if selected_text else query
            search_results = await self._search_qdrant(search_query)

        if not search_results:
            return "I could not find any relevant information to answer your question.", []

        context_str = ""
        sources = set()
        for result in search_results:
            source = result['payload'].get('source', 'N/A')
            content = result['payload'].get('page_content', result['payload'].get('text', 'N/A'))
            context_str += f"Source: {source}\n"
            context_str += f"Content: {content}\n---\n"
            sources.add(source)

        try:
            chain = self.rag_prompt | self.llm
            response = await chain.ainvoke({
                "context": context_str,
                "question": query,
            })

            answer = response.content

            if "Sources:" not in answer:
                answer += "\n\n**Sources:**\n" + "\n".join(f"- {s}" for s in sorted(list(sources)))

            return answer, sorted(list(sources))
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            raise

# --- Singleton Management ---
rag_agent_instance = None

def get_rag_agent():
    """
    Initializes and returns a singleton instance of the RAGAgent.
    This lazy initialization ensures that environment variables are loaded
    before the agent is created.
    """
    global rag_agent_instance
    if rag_agent_instance is None:
        logger.info("Initializing RAGAgent singleton...")
        rag_agent_instance = RAGAgent()
    return rag_agent_instance

async def process_query(query: str, selected_text: str = None) -> Tuple[str, List[str]]:
    """
    Processes a user query by calling the RAGAgent instance.
    This maintains compatibility with the existing router setup.
    """
    try:
        agent = get_rag_agent()
        return await agent.generate_response(query=query, selected_text=selected_text)
    except Exception as e:
        logger.error(f"Error in process_query: {e}")
        raise
