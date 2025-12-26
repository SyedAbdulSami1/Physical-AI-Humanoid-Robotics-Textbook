# ADR-0006: Text Embedding Model

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** The RAG pipeline's retrieval quality heavily depends on the quality of the vector embeddings. The chosen embedding model needs to be effective at capturing the semantic meaning of technical content, including code snippets and specialized terminology found in the robotics and AI textbook. The decision involves a trade-off between performance, cost, and ease of use.

## Decision

We will use **Google Gemini's `embedding-001` model**. It is a high-performing, free-tier model that provides a strong baseline for semantic search. Its integration is straightforward via the Google Gemini API, which simplifies development and aligns with the project's goal of using free-tier services.

## Consequences

### Positive

- **High Quality:** `embedding-001` is a powerful model suitable for technical content.
- **Ease of Use:** As a managed API, there is no need to host or maintain the model. Integration is a simple API call.
- **Free Tier:** Aligns with the project's constraint of using free-tier services.

### Negative

- **Vendor Lock-in:** The application becomes dependent on the Google Gemini API. Switching to another provider would require code changes.
- **Proprietary:** The model is a black box, giving us no control over its architecture or training data.

## Alternatives Considered

- **Open-Source Sentence Transformers (e.g., `all-MiniLM-L6-v2`):** Hosting a smaller, open-source model locally as part of the backend service.
    - **Why Rejected:** While this would be free to run (post-setup), it introduces significant complexity. We would need to manage the model's lifecycle, and the chosen model (`all-MiniLM-L6-v2`) while good, is generally less powerful than `ada-002`. The engineering effort for hosting and the potential hit to retrieval quality make it less suitable for a rapid-turnaround hackathon.

- **Other Managed Embedding APIs (e.g., Cohere):**
    - **Why Rejected:** Similar to the choice of LLM, sticking with a single primary provider (Google Gemini) for both embeddings and generation simplifies API key management and billing for this initial phase.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
