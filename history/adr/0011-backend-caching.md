# ADR-0011: Backend Caching

- **Status:** Proposed
- **Date:** 2025-12-30
- **Feature:** textbook-physical-ai
- **Context:** The backend service will handle computationally expensive RAG queries. To improve performance and reduce redundant processing, a caching layer is being considered for frequently accessed content and repeated queries.

## Decision

We will implement a **simple in-memory cache** within the FastAPI application using a library like `cachetools`. This will provide an immediate performance boost for repeated requests, such as frequently asked questions or commonly accessed content chunks. We will use a time-to-live (TTL) cache to ensure that cached results expire after a certain period, allowing for content updates to be reflected. For a hackathon-scale project, an in-memory solution is sufficient and avoids the complexity of an external caching service.

## Consequences

### Positive

- **Improved Performance:** Caching will significantly speed up responses for repeated requests, especially for frequently accessed content.
- **Reduced Computational Load:** It will decrease the number of vector similarity searches and LLM calls for repeated queries, saving computational resources.
- **Simplicity:** An in-memory cache is easy to implement and manage within the application code, requiring no external services.

### Negative

- **Increased Memory Usage:** The cache will consume memory on the backend server. A large cache could impact the application's overall performance.
- **Not Persistent:** The cache is ephemeral and will be lost if the server restarts. This is acceptable for this project but would not be suitable for a production system requiring a durable cache.
- **Not Distributed:** In a multi-instance deployment, each instance would have its own separate cache, leading to duplicated effort. This is not a concern for our single-instance deployment.

## Alternatives Considered

- **Redis:** Using an external Redis instance as a dedicated caching server.
    - **Why Rejected:** Setting up and managing a Redis server (even a managed one) adds significant complexity and another infrastructure component to manage. For the scale of this project, it is overkill. The benefits of a distributed, persistent cache do not outweigh the setup costs for a hackathon.
- **No Caching:** Not implementing any caching and simply processing every request as it comes in.
    - **Why Rejected:** This would lead to a poor user experience, with unnecessarily long waits for repeated requests. It's also an inefficient use of compute resources, which could be a problem in a resource-constrained free-tier environment.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
