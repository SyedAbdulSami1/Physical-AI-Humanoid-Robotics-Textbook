# app/skills/rag_agent.py

import os
from typing import List, Tuple
from qdrant_client import QdrantClient, models as qdrant_models
from langchain_openai import OpenAIEmbeddings, ChatOpenAI
from langchain.prompts import ChatPromptTemplate

# --- Client and Model Setup ---

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
QDRANT_COLLECTION_NAME = "textbook_content"

# Initialize clients and models
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
embeddings = OpenAIEmbeddings(api_key=OPENAI_API_KEY)
llm = ChatOpenAI(model="gpt-4-turbo", temperature=0.1, api_key=OPENAI_API_KEY)

# --- Prompt Template ---

RAG_PROMPT_TEMPLATE = """
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

rag_prompt = ChatPromptTemplate.from_template(RAG_PROMPT_TEMPLATE)

async def search_qdrant(query: str, top_k: int = 5) -> List[dict]:
    """
    Searches the Qdrant collection for the most relevant document chunks.

    Args:
        query: The user's query.
        top_k: The number of results to retrieve.

    Returns:
        A list of search results.
    """
    vector = embeddings.embed_query(query)
    search_results = qdrant_client.search(
        collection_name=QDRANT_COLLECTION_NAME,
        query_vector=vector,
        limit=top_k,
        with_payload=True,
    )
    return [result.model_dump() for result in search_results]

async def process_query(query: str, selected_text: str = None) -> Tuple[str, List[str]]:
    """
    Processes a user query through the RAG pipeline.

    Args:
        query: The user's question.
        selected_text: Optional text selected by the user to narrow the search.

    Returns:
        A tuple containing the generated answer and a list of source documents.
    """
    # If user selected text, combine it with the query for a more focused search
    search_query = f"{selected_text}\n\n{query}" if selected_text else query

    # 1. Retrieve context from Qdrant
    search_results = await search_qdrant(search_query)

    if not search_results:
        return "I could not find any relevant information to answer your question.", []

    # 2. Format the context and collect sources
    context_str = ""
    sources = set()
    for result in search_results:
        context_str += f"Source: {result['payload']['source']}\n"
        context_str += f"Content: {result['payload'].get('page_content', 'N/A')}\n---\n"
        sources.add(result['payload']['source'])

    # 3. Generate the answer using the LLM
    chain = rag_prompt | llm
    
    response = await chain.ainvoke({
        "context": context_str,
        "question": query,
    })

    answer = response.content
    
    # Append sources to the answer if not already there (belt-and-suspenders)
    if "Sources:" not in answer:
        answer += "\n\n**Sources:**\n" + "\n".join(f"- {s}" for s in sorted(list(sources)))

    return answer, sorted(list(sources))
