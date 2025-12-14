# app/skills/rag_agent.py

import os
from typing import List, Tuple
from qdrant_client import QdrantClient, models as qdrant_models
from langchain_openai import OpenAIEmbeddings, ChatOpenAI
from langchain.prompts import ChatPromptTemplate

class RAGAgent:
    def __init__(self):
        # --- Client and Model Setup ---
        self.QDRANT_URL = os.getenv("QDRANT_URL")
        self.QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
        self.OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
        self.QDRANT_COLLECTION_NAME = "textbook_content"

        # Initialize clients and models
        self.qdrant_client = QdrantClient(url=self.QDRANT_URL, api_key=self.QDRANT_API_KEY)
        self.embeddings = OpenAIEmbeddings(api_key=self.OPENAI_API_KEY)
        self.llm = ChatOpenAI(model="gpt-4-turbo", temperature=0.1, api_key=self.OPENAI_API_KEY)

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
        vector = self.embeddings.embed_query(query)
        search_results = self.qdrant_client.search(
            collection_name=self.QDRANT_COLLECTION_NAME,
            query_vector=vector,
            limit=top_k,
            with_payload=True,
        )
        return [result.model_dump() for result in search_results]

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

        chain = self.rag_prompt | self.llm
        
        response = await chain.ainvoke({
            "context": context_str,
            "question": query,
        })

        answer = response.content
        
        if "Sources:" not in answer:
            answer += "\n\n**Sources:**\n" + "\n".join(f"- {s}" for s in sorted(list(sources)))

        return answer, sorted(list(sources))

# Create a singleton instance of the RAGAgent
rag_agent_instance = RAGAgent()

async def process_query(query: str, selected_text: str = None) -> Tuple[str, List[str]]:
    """
    Processes a user query by calling the RAGAgent instance.
    This maintains compatibility with the existing router setup.
    """
    return await rag_agent_instance.generate_response(query=query, selected_text=selected_text)
