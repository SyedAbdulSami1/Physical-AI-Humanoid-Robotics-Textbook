# ADR-0003: Relational Database Provider

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** The project requires a relational database to store user data, including profiles from the background questionnaire and conversation histories for the chatbot. The solution needs to be low-cost (ideally free for the hackathon), easy to manage, and compatible with a Python backend (FastAPI) using an ORM like SQLAlchemy.

## Decision

We will use **Neon Serverless Postgres (Free Tier)** as our relational database provider. Neon offers a fully managed, serverless PostgreSQL database. Its serverless nature means it automatically scales to zero when not in use, making it extremely cost-effective. The free tier is generous and more than sufficient for storing user and conversation data for this project. As it's standard Postgres, it has excellent compatibility with SQLAlchemy.

## Consequences

### Positive

- **Cost-Efficiency:** The serverless "scale-to-zero" model is perfect for a project with intermittent traffic like this hackathon, ensuring we only pay for what we use (which is nothing on the free tier).
- **Fully Managed:** Neon handles all database administration tasks, including backups, scaling, and security, freeing up the team to focus on application logic.
- **Standard PostgreSQL:** No custom API or fork. We can use the standard, battle-tested PostgreSQL ecosystem, including libraries like `psycopg2` and `SQLAlchemy`.
- **Separation of Compute and Storage:** This architecture provides high availability and efficient resource usage.

### Negative

- **Cold Starts:** The first request after a period of inactivity might experience a "cold start" delay as the database compute resource spins up. This is a common tradeoff for serverless architectures.
- **Vendor Lock-in:** While it's standard Postgres, migrating the managed infrastructure and Neon-specific features (like branching) to another provider would require effort.

## Alternatives Considered

- **Supabase:** Supabase provides a suite of backend tools, including a managed Postgres database, authentication, and storage.
    - **Why Rejected:** Supabase is an excellent platform but provides more than we need. We are building our own authentication and backend logic with FastAPI, so the additional features (like Auth and storage) would be unused. Neon is a more focused "database-as-a-service" that fits our specific need without extra complexity.

- **Self-hosted PostgreSQL (e.g., in Docker):** Running our own Postgres instance.
    - **Why Rejected:** Similar to the reasoning for the vector database, self-hosting introduces significant operational complexity (setup, backups, security, resource management) that is undesirable and unnecessary for a rapid-development hackathon project.

- **SQLite:** Using a file-based database.
    - **Why Rejected:** While simple to set up, SQLite is not well-suited for concurrent write access from a web server and is harder to manage, inspect, and scale compared to a proper client-server database like Postgres.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
