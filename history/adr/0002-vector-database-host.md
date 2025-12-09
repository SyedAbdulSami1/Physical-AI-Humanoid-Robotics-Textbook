# ADR-0002: Vector Database Host

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** The RAG (Retrieval-Augmented Generation) chatbot requires a vector database to store embeddings of the textbook content for efficient semantic search. The choice of hosting will impact scalability, maintenance overhead, and cost. The project is a hackathon prototype, so cost-effectiveness and ease of setup are primary drivers.

## Decision

We will use **Qdrant Cloud (Free Tier)** as the vector database host. Qdrant Cloud provides a managed, cloud-native vector database solution that is optimized for performance and scalability. The free tier is sufficient for the project's needs, offering enough storage and compute for the textbook's content. This choice offloads the operational burden of managing and scaling the database.

## Consequences

### Positive

- **Zero-Maintenance:** As a fully managed service, Qdrant Cloud eliminates the need for us to handle setup, scaling, and maintenance of the database infrastructure.
- **Scalability:** Qdrant Cloud is designed to scale, so if the project grows, we can easily upgrade to a paid tier without a complex migration.
- **Cost-Effective:** The free tier provides sufficient resources for the scope of this hackathon project, incurring no costs.
- **Performance:** Qdrant is a high-performance vector search engine written in Rust, ensuring fast and accurate retrieval for the RAG pipeline.

### Negative

- **Network Latency:** Being a cloud service, there will be network latency between our backend and the database, although this is generally minimal.
- **Vendor Lock-in:** We become dependent on Qdrant's service and API. Migrating to another vector database in the future would require effort.
- **Data Privacy:** The textbook content will be stored on a third-party cloud service. While Qdrant is a reputable provider, this is a consideration for sensitive data (though our content is public).

## Alternatives Considered

- **Local Qdrant Instance (e.g., in Docker):** Running a Qdrant instance locally within our backend infrastructure.
    - **Why Rejected:** This would introduce significant operational overhead. We would be responsible for setup, resource management (CPU/RAM), data persistence, and backups. For a hackathon, this adds unnecessary complexity and distracts from the core application development.

- **Other Managed Vector Databases (e.g., Pinecone, Weaviate Cloud):** These are other popular managed vector database providers.
    - **Why Rejected:** Qdrant was chosen due to its performance, open-source nature (which aligns with the overall project ethos), and a generous free tier that perfectly fits the project's requirements. While others are viable, Qdrant meets all our needs with no significant downside for this use case.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
