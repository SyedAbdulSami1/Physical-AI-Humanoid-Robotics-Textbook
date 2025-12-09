# ADR-0004: Authentication Service

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** The application requires a user authentication system to manage user profiles, store personalization preferences (from the background questionnaire), and gate access to certain features like conversation history. The solution must be secure, easy to integrate with a FastAPI backend and a React frontend, and quick to set up for a hackathon.

## Decision

We will use **Better-Auth** for handling user authentication. Better-Auth is a developer-friendly authentication service that provides a simple and secure way to add login, signup, and user management to an application. Its focus on simplicity and ease of integration makes it ideal for a fast-paced hackathon project. We will integrate it into our FastAPI backend.

## Consequences

### Positive

- **Rapid Implementation:** Better-Auth is designed for quick setup, allowing us to implement a full authentication system in a fraction of the time it would take to build one from scratch.
- **Security:** By outsourcing authentication, we leverage the expertise of a dedicated service, reducing the risk of security vulnerabilities in our own code. It handles password hashing, token management, and other security-critical aspects.
- **Focus on Core Features:** We can focus on building the core textbook and chatbot features instead of spending time on the complexities of authentication.
- **Decoupled:** The service is independent of our main application stack, which is a good architectural practice.

### Negative

- **External Dependency:** Our application's user management will be dependent on an external, third-party service. If Better-Auth has an outage, our login/signup will be affected.
- **Vendor Lock-in:** Migrating away from Better-Auth to another authentication service or a self-hosted solution in the future would require refactoring our authentication logic.
- **Data Resides Externally:** User profile data (like email and credentials) will be stored on Better-Auth's servers, which may be a consideration for data governance policies in a real-world product.

## Alternatives Considered

- **Clerk:** Another popular authentication and user management service.
    - **Why Rejected:** Clerk is a very strong contender, but Better-Auth is perceived as being even simpler and more lightweight for the specific needs of this hackathon, where speed is a primary concern. Both are excellent choices.
- **Custom FastAPI Implementation (with JWT):** Building our own authentication logic using libraries like `passlib` for hashing and `python-jose` for JWT creation and verification within our FastAPI backend.
    - **Why Rejected:** This approach is time-consuming and fraught with potential security risks if not implemented perfectly. For a hackathon, the time investment required to build a secure, custom authentication solution is not justifiable. It would divert significant effort from the core product features.
- **Supabase Auth:** Using the built-in authentication feature of Supabase.
    - **Why Rejected:** As mentioned in ADR-0003, we chose not to use the full Supabase stack to keep our architecture more focused. Using only its auth component while using Neon for the database would create an unnecessary proliferation of backend-as-a-service providers.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
