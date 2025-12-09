# ADR-0012: "Selected Text Only" RAG Logic

- **Status:** Proposed
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** A key feature of the RAG chatbot is its ability to answer questions based on a specific piece of text that the user has highlighted on the page. This requires a mechanism to isolate the user's selection and use it as the sole context for the LLM's response, bypassing the larger vector store retrieval process.

## Decision

We will implement this feature by creating a **separate, dedicated RAG chain or flow in the backend**. When the user invokes the "selected text" mode, the frontend will send the highlighted text along with the query to a specific API endpoint (e.g., `/rag/selected`). The backend will then bypass the Qdrant vector search entirely. Instead, it will treat the provided text selection as the *only* context. This context will be directly injected into the prompt that is sent to the LLM, along with the user's question. This ensures the LLM's response is strictly limited to the provided text.

## Consequences

### Positive

- **Precision:** This approach guarantees that the chatbot's answer is based only on the user-selected text, providing a highly focused and predictable tool for text-specific inquiries.
- **Simplicity:** The logic is straightforward to implement. It avoids complex filtering or temporary indexing in the vector store. It's a clean separation from the main RAG pipeline.
- **Performance:** For "selected text" queries, we skip the potentially time-consuming vector search, which could lead to faster response times.

### Negative

- **Loss of Broader Context:** By design, the chatbot will not have access to the wider context of the book. If the user's question requires information outside the selected snippet, the chatbot will be unable to answer it effectively.
- **Requires Clear UI/UX:** The user interface must make it very clear when they are in "selected text" mode versus the normal "full book" RAG mode to avoid confusion.

## Alternatives Considered

- **Prompt Engineering with a Single Endpoint:** Using the same RAG endpoint but trying to control the context via prompt engineering. For example, sending a prompt like: "Based *only* on the following text: [selected text], answer the question: [question]".
    - **Why Rejected:** This is brittle and relies on the LLM to perfectly follow the instruction to ignore its other retrieved context. LLMs can sometimes "leak" information from the broader context provided to them, even when instructed not to. A separate, dedicated logic path is more robust and guarantees isolation.
- **Temporary Vector Store/Indexing:** Creating a temporary, in-memory vector store containing only the selected text and performing the RAG process against that.
    - **Why Rejected:** This is overly complex and inefficient. It involves embedding the selected text, setting up a temporary vector store, and then performing a search—all for a single query on a tiny amount of text. Directly using the text as context is far simpler and more performant.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
