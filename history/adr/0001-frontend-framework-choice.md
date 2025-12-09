# ADR-0001: Frontend Framework Choice

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** The project requires a framework to build the user-facing textbook website. The framework must be suitable for a content-heavy site, allow for easy integration of custom React components (for the RAG chatbot), and support a static-first architecture for performance and simple deployment. The primary goal is to present Markdown-based content in a clean, readable, and interactive format.

## Decision

We will use **Docusaurus** as the frontend framework. Docusaurus is a static site generator specifically designed for building documentation websites, which aligns perfectly with the textbook format. It provides excellent Markdown support out-of-the-box, a plugin-based architecture, and is built on React, allowing for seamless integration of our custom chatbot component.

## Consequences

### Positive

- **Optimized for Content:** Docusaurus is purpose-built for content-driven sites, providing features like versioning, documentation sidebars, and search out of the box.
- **React-based:** Allows for the use of the entire React ecosystem and simplifies the integration of the React-based chatbot UI.
- **Strong Community & Tooling:** Benefits from a large community, good documentation, and a mature toolchain.
- **Static Generation:** Produces a highly performant static site that can be easily deployed and scaled on services like GitHub Pages or Vercel.
- **Fast Development:** Rapidly create content-focused pages and structures.

### Negative

- **Opinionated Structure:** Docusaurus enforces a specific project structure, which might be less flexible than a general-purpose framework like Next.js for non-documentation type pages.
- **Less Suited for App-like Complexity:** While it supports React, it's not primarily designed for complex, state-heavy single-page applications. If the interactive components become very complex, we might hit limitations.

## Alternatives Considered

- **Next.js with MDX:** Next.js is a powerful, full-featured React framework. Using it with MDX would allow us to embed React components in Markdown.
    - **Why rejected:** This approach would require more manual setup to replicate the features Docusaurus provides out-of-the-box (e.g., sidebars, versioning, docs-specific SEO). It's more of a general-purpose web application framework, making it overkill and more complex for a content-centric site.

- **VitePress:** A Vue-based static site generator, similar in purpose to Docusaurus.
    - **Why rejected:** The team has stronger expertise in React, making Docusaurus a more natural fit. Sticking to a single framework (React) for both the site and the interactive components reduces cognitive overhead.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
