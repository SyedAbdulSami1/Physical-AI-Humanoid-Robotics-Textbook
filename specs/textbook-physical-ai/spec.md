# Specification: Physical AI & Humanoid Robotics Textbook

## Intent
The intent is to deliver an AI-Native Textbook on Physical AI and Humanoid Robotics, integrated into a publicly deployed Docusaurus site. This project, a deliverable for the Panaversity Hackathon I, will include core features such as a RAG chatbot and will be developed using Spec-Kit Plus. Additionally, it will incorporate bonus features like user authentication with personalization, and on-demand Urdu translation.

## User Personas
*   **Students/Learners**: Individuals seeking to learn about Physical AI and Humanoid Robotics, who are assumed to have basic Python knowledge (syntax, data structures, functions). This persona ranges from beginners in robotics to advanced practitioners, all benefiting from structured content, code examples, diagrams, exercises, and further reading.
*   **Researchers/Developers**: Professionals and enthusiasts looking for in-depth information, practical code implementations, and a reliable reference for Physical AI concepts; their experience will include access to code-centric views and advanced examples.
*   **Global Audiences**: Users who require content in languages other than English, specifically Urdu, to facilitate their learning and research.
*   **Personalized Learners**: Users who prefer tailored educational content based on their existing software and hardware experience.

## Success Evals
*   The textbook structure comprises exactly 13 or more chapters, adhering precisely to the predefined weekly breakdown.
*   Each chapter within the textbook consistently integrates relevant code examples, explanatory diagrams, practical exercises, and suggestions for further reading.
*   The live Docusaurus site is publicly accessible and functional at the specified GitHub Pages URL: `<username>.github.io/<repo>`.
*   The embedded RAG chatbot demonstrates high accuracy, correctly answering at least 95% of a set of 20 predefined test questions based solely on the textbook's content.
*   The chatbot's functionality extends to accurately answering questions when users highlight and select specific text within the textbook as context.
*   The RAG chatbot must clearly inform the user if it cannot find relevant information within the textbook's content for a given query, instead of attempting to answer from external knowledge.
*   The on-demand Urdu translation feature is operational across all chapters, providing translations with a latency not exceeding 3 seconds per request, and is triggered by a site-wide language toggle switch in the main navigation bar.
*   The "Personalize Content" button successfully injects beginner-friendly contextual notes (e.g., definitions, simpler explanations) into a minimum of 30% of a chapter's content, based on the user's stored profile. User profile data for personalization is stored in browser local storage/cookies, implying stateless personalization across devices and potential data loss upon browser cache clearing.
*   The entire project's source code is licensed under the MIT License, exclusively utilizes free-tier cloud services, and contains no hard-coded secrets.
*   Full Spec-Kit Plus artifacts, including Architecture Decision Records (ADRs), Product Health Reports (PHRs), and historical records, are fully preserved within the project.
*   The completed project is submitted before the deadline of Sunday, November 30, 2025, 06:00 PM PKT.

## Constraints
*   The project must exclusively rely on free-tier services for all deployed components and infrastructure.
*   The Docusaurus site implementation must utilize Docusaurus v3 or a newer version, and include essential features such as dark mode, integrated search functionality, and content versioning.
*   All code examples and practical guidance for robotics must be based on ROS 2 LTS (e.g., Jazzy) for stability and long-term support.
*   All simulation examples must use a recent stable version of Isaac Sim (e.g., 2024.x).
*   All Urdu translations provided by the system, utilizing a local open-source model (e.g., NLLB on Hugging Face), must achieve a high level of linguistic quality, comparable to human translation, and avoid the characteristics of poor machine translation. The local model must operate within free-tier service limitations.
*   The RAG chatbot will utilize a local open-source LLM (e.g., Llama 3, Mistral, Gemma) to adhere to free-tier constraints and minimize external API dependencies.
*   The Docusaurus site must be fully responsive, ensuring optimal user experience and display across both mobile and desktop devices.
*   The total size of the GitHub repository, including all code, documentation, and assets, must not exceed 2 GB.
*   Upon failure of the external translation model, the system must display an error message and revert to the original English language, ensuring content accessibility.

## Non-Goals
*   Integration with physical robot hardware or control systems is explicitly out of scope.
*   The creation or embedding of video lectures or YouTube content is not part of this project.
*   No payment or subscription system for accessing content or features will be implemented.
*   Real-time collaborative editing functionalities for textbook content are excluded.
*   Development of an offline Progressive Web Application (PWA) is not a project goal.
*   The capability to export textbook content to PDF format is not included.
*   Implementation of a discussion forum or community interaction features is outside the project scope.

## Clarifications
### Session 2025-12-08
- Q: How should the "Personalize Content" feature concretely modify a chapter for a user profiled as a 'beginner'? → A: Add contextual notes: Inject small, callout boxes (`<Info>...</Info>`) with beginner-friendly definitions or simpler explanations above complex code or math.
- Q: If the RAG chatbot finds no relevant information in the textbook for a given question, how must it respond? → A: State it cannot answer: Clearly inform the user that the answer is not in the textbook's content.
- Q: What is the single most important prerequisite skill or knowledge the textbook should assume the reader *already has*? → A: Basic Python knowledge: Assume familiarity with Python syntax, basic data structures, and functions.
- Q: What specific versions of ROS 2 and Isaac Sim should the textbook's code and examples be based on? → A: ROS 2 LTS (e.g., Jazzy) & Recent Stable Isaac Sim (e.g., 2024.x): Prioritize stability and long-term support for ROS 2, and a recent, well-tested version for Isaac Sim.
- Q: For the Urdu translation feature, what is the required implementation method? → A: Local model (e.g., NLLB on Hugging Face): Integrate an open-source, locally run model for translation.
### Session 2025-12-09
- Q: How does the user trigger the on-demand Urdu translation for a piece of content? → A: A site-wide language toggle switch in the main navigation bar.
- Q: Where will user profile data for the personalization feature be stored? → A: Browser Local Storage / Cookies.
- Q: What is the underlying Large Language Model (LLM) service for the RAG chatbot? → A: Local Open-Source LLM (e.g., Llama 3, Mistral, Gemma).
- Q: Besides beginner notes, how should the experience differ for a "Researcher/Developer" versus a "Student/Learner"? → A: Code-centric views/advanced examples.
- Q: How should the system handle a failure of the external translation model? → A: Display error message and revert to original language.

## Constitution Alignment Check
The project strongly aligns with the constitution's principles of:
*   **Knowledge Dissemination**: Creating an AI-native textbook fosters education and widespread understanding in a critical emerging field.
*   **Open Source Contribution**: Adherence to the MIT license promotes collaboration and accessibility of the codebase.
*   **AI-Driven Innovation**: Leveraging AI for content personalization and RAG chatbot functionality exemplifies innovative application of AI for enhanced learning experiences.
*   **Structured Development**: The mandated use of Spec-Kit Plus ensures a systematic, transparent, and robust development process, aligning with best practices for project governance and maintainability.
*   **Accessibility and Inclusivity**: Providing on-demand Urdu translation supports a broader, more inclusive global audience.

---

## RAG Chatbot Implementation Requirements

### Technical Architecture

#### Backend Requirements
*   **Framework**: The RAG chatbot backend MUST utilize FastAPI as the primary web framework, with all endpoints implemented as asynchronous functions (`async def`) to ensure optimal performance and scalability.
*   **Database**: Neon Serverless Postgres MUST be used for all user/session storage and personalization data, including user profiles, authentication sessions, personalization preferences, and background questionnaire responses.
*   **Vector Storage**: Qdrant Cloud Free Tier MUST be used for vector embeddings storage and retrieval, containing embeddings of all textbook content for accurate semantic search and retrieval.
*   **Generative AI**: OpenAI/ChatKit SDKs MUST be leveraged for content generation and response formulation, properly integrating retrieved context from the vector store with the LLM's generative capabilities.

#### RAG Functionality
*   **Full Book Querying**: The chatbot MUST be able to answer questions based on the entire content of the textbook with high accuracy.
*   **Selected-Text-Only Mode**: The chatbot MUST support a selected-text-only RAG mode where user queries are contextualized based only on highlighted text sections rather than the entire textbook corpus. This functionality restricts answers strictly to user-selected chunks.
*   **Ingestion Pipeline**: An automated ingestion pipeline MUST be created to read all book Markdown content, chunk content appropriately, generate embeddings, and persist them to the vector store during build/deploy with error handling and validation.

#### User Authentication and Personalization
*   **Authentication**: Better-Auth integration MUST be implemented with mandatory background questionnaire at user signup. The questionnaire MUST collect information about the user's software/hardware experience to enable content personalization.
*   **Personalization Logic**: Per-chapter "Personalize" button functionality MUST be implemented using stored user profiles. The content adaptation MUST be dynamic and tailored to the user's experience level as indicated in their profile (e.g., showing beginner vs advanced explanations based on user profile).

#### Translation Capabilities
*   **Urdu Translation**: Per-chapter "Translate to Urdu" button MUST be implemented with a reliable translation service that maintains technical accuracy and readability. The translation service MUST handle technical terminology appropriately.

### User Interface and Experience

#### Frontend Components
*   **Docusaurus Integration**: All interactive features (chatbot, auth forms, personalize button, translate button) MUST be implemented as modular Docusaurus/React components and seamlessly integrated into the site.
*   **Loading States**: All interactive features MUST include appropriate loading states to indicate processing to the user.
*   **Fallback Mechanisms**: The system MUST include fallback mechanisms for when external services fail (e.g., translation service unavailable).

### Quality and Performance Requirements

#### Performance Standards
*   **Async Endpoints**: All backend endpoints MUST be implemented asynchronously using `async def` functions for all chat queries, personalization, translation, user profile management, and content retrieval.
*   **Authentication Middleware**: All endpoints that handle user data, personalization, or content modification MUST be protected by authentication middleware with access control enforced based on user roles and session validity.
*   **Error Handling**: Comprehensive error handling MUST be implemented throughout the application with graceful degradation in response to service failures (e.g., LLM unavailability, vector store downtime).

#### Quality Gates
*   **RAG Accuracy**: The RAG system MUST achieve >90% accuracy on test queries, with retrieval accuracy exceeding 90% for test queries.
*   **Testing Suite**: A comprehensive test suite MUST be implemented to measure RAG accuracy, validating that the system retrieves relevant context and generates correct answers with acceptable precision and recall metrics.
*   **Reusable Skills**: Reusable skills and subagents MUST be created specifically for chatbot workflows, with complex operations like RAG retrieval, content personalization, and translation encapsulated in modular, testable components.

### Success Criteria for RAG Implementation
*   The RAG chatbot correctly answers at least 90% of test questions based solely on textbook content
*   Selected-text-only mode functions with >85% accuracy when users highlight specific content sections
*   Per-chapter "Personalize" button dynamically adapts content to user experience level within 2 seconds
*   Urdu translation completes with <3 second latency and maintains technical accuracy >90%
*   Ingestion pipeline processes all book content within 10 minutes during deployment
*   System maintains sub-2 second response times for 95% of requests under normal load
*   Authentication and personalization features are available to 100% of registered users
*   All backend endpoints respond asynchronously with appropriate error handling
*   The system gracefully handles service outages without crashing
*   Reusable skills/subagents demonstrate clear separation of concerns and testability