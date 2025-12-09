# ADR-0010: Chat UI Library

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** The RAG chatbot requires a user interface component to be embedded within the Docusaurus site. The decision is whether to build this chat UI from scratch using React components or to adopt a pre-built library to accelerate development. The UI needs to support message display, user input, and loading states.

## Decision

We will use a pre-built React chat component library, specifically **ChatKit**, to create the chatbot UI. This will provide us with a ready-made, customizable set of components for building a chat interface, including message lists, input fields, and user avatars. Using a library like this will dramatically speed up frontend development.

## Consequences

### Positive

- **Accelerated Development:** Saves a significant amount of time compared to building a chat interface from the ground up. We can focus on the logic of the chatbot rather than the UI details.
- **Feature-Rich:** These libraries typically come with many built-in features, such as typing indicators, message timestamps, and theming, which would be time-consuming to build ourselves.
- **Professional Appearance:** Provides a polished, professional-looking chat interface out of the box.

### Negative

- **Less Flexibility:** We are constrained by the customization options provided by the library. Highly specific or unconventional UI designs might be difficult or impossible to implement.
- **Added Dependency:** Introduces another third-party dependency into the project, which needs to be maintained and could have its own bugs or limitations.
- **Potential for Code Bloat:** The library might include more features and code than we strictly need, potentially increasing the overall bundle size of the application.

## Alternatives Considered

- **Build Custom Components:** Creating our own chat UI from scratch using basic React components and state management.
    - **Why Rejected:** This would be a significant undertaking for a hackathon. Building a robust and polished chat UI involves many small details (e.g., scrolling behavior, input handling, message styling) that would consume a large amount of development time, detracting from the core AI and content features of the project.
- **Other Chat UI Libraries (e.g., react-chat-elements):** There are other libraries available for building chat interfaces.
    - **Why Rejected:** ChatKit is a well-regarded and feature-complete option. While other libraries are also suitable, ChatKit provides a good balance of features, customization, and ease of use that is well-suited for this project. There is no compelling reason to choose a different one at this stage.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
