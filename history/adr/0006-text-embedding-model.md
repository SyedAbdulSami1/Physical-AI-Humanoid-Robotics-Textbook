# ADR-0006: Text Embedding Model

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** The RAG pipeline's retrieval quality heavily depends on the quality of the vector embeddings. The chosen embedding model needs to be effective at capturing the semantic meaning of technical content, including code snippets and specialized terminology found in the robotics and AI textbook. The decision involves a trade-off between performance, cost, and ease of use.

## Decision

We will start with **OpenAI's `text-embedding-ada-002` model**. It is a widely-used, high-performing model that provides a strong baseline for semantic search. Its integration is straightforward via the OpenAI API, which simplifies development. While newer models exist, `ada-002` offers a proven balance of quality and cost, making it a safe and effective choice for this project. As the project evolves, we can benchmark and potentially upgrade to a newer or more specialized model if required.

## Consequences

### Positive

- **High Quality:** `text-embedding-ada-002` is a mature and powerful model known for producing high-quality embeddings for a wide range of text types.
- **Ease of Use:** As a managed API, there is no need to host or maintain the model. Integration is a simple API call.
- **Strong Ecosystem:** It is well-supported by libraries like `langchain` and has extensive documentation.

### Negative

- **Cost:** Using the OpenAI API incurs a cost based on the number of tokens embedded. For a large book and frequent updates, this could become a factor.
- **Vendor Lock-in:** The application becomes dependent on the OpenAI API. Switching to another provider would require code changes.
- **Proprietary:** The model is a black box, giving us no control over its architecture or training data.

## Alternatives Considered

- **Open-Source Sentence Transformers (e.g., `all-MiniLM-L6-v2`):** Hosting a smaller, open-source model locally as part of the backend service.
    - **Why Rejected:** While this would be free to run (post-setup), it introduces significant complexity. We would need to manage the model's lifecycle, and the chosen model (`all-MiniLM-L6-v2`) while good, is generally less powerful than `ada-002`. The engineering effort for hosting and the potential hit to retrieval quality make it less suitable for a rapid-turnaround hackathon.
- **Newer OpenAI Models (e.g., `text-embedding-3-small` / `large`):** Using OpenAI's latest generation of embedding models.
    - **Why Rejected (for now):** `ada-002` is sufficient and a well-understood baseline. While newer models offer potential improvements (e.g., performance, cost-effectiveness, and dimensionality reduction), starting with the established `ada-002` is a lower-risk choice. We can easily upgrade later if performance metrics show a need. This defers a decision until more data is available.
- **Other Managed Embedding APIs (e.g., Cohere):**
    - **Why Rejected:** Similar to the choice of LLM, sticking with a single primary provider (OpenAI) for both embeddings and generation simplifies API key management and billing for this initial phase.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
