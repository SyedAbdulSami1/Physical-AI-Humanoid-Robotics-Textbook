# Specification: Physical AI & Humanoid Robotics Textbook

## Intent
The intent is to deliver an AI-Native Textbook on Physical AI and Humanoid Robotics, integrated into a publicly deployed Docusaurus site. This project, a deliverable for the Panaversity Hackathon I, will include core features such as a public RAG chatbot with a modernized UI and will be developed using Spec-Kit Plus. All user authentication has been removed to make the chatbot publicly accessible. The project exclusively uses the free Google Gemini API and does not include any translation features to maintain 100% free operation.

## User Personas
*   **Students/Learners**: Individuals seeking to learn about Physical AI and Humanoid Robotics, who are assumed to have basic Python knowledge (syntax, data structures, functions). This persona ranges from beginners in robotics to advanced practitioners, all benefiting from structured content, code examples, diagrams, exercises, and further reading.
*   **Researchers/Developers**: Professionals and enthusiasts looking for in-depth information, practical code implementations, and a reliable reference for Physical AI concepts; their experience will include access to code-centric views and advanced examples.


## Success Evals
*   The textbook structure comprises exactly 13 or more chapters, adhering precisely to the predefined weekly breakdown.
*   Each chapter within the textbook consistently integrates relevant code examples, explanatory diagrams, practical exercises, and suggestions for further reading.
*   The live Docusaurus site is publicly accessible and functional at the specified GitHub Pages URL: `<username>.github.io/<repo>`.
*   The embedded RAG chatbot demonstrates high accuracy, correctly answering at least 95% of a set of 20 predefined test questions based solely on the textbook's content.
*   The chatbot's functionality extends to accurately answering questions when users highlight and select specific text within the textbook as context.
*   The RAG chatbot must clearly inform the user if it cannot find relevant information within the textbook's content for a given query, instead of attempting to answer from external knowledge.
*   The Docusaurus UI has been modernized to be clean, modern, readable, and professional. This includes improvements to typography, layout, spacing, and navigation. The chatbot UI has also been improved with better message bubbles, responsiveness, and dark/light mode support.
*   The RAG chatbot is fully public and requires no authentication.
*   The project exclusively uses the free Google Gemini API with no paid features or services.
*   Translation features have been removed to maintain 100% free operation.
*   Full Spec-Kit Plus artifacts, including Architecture Decision Records (ADRs), Product Health Reports (PHRs), and historical records, are fully preserved within the project.
*   The completed project is submitted before the deadline of Sunday, November 30, 2025, 06:00 PM PKT.

## Constraints
*   The project must exclusively rely on free-tier services for all deployed components and infrastructure.
*   The Docusaurus site implementation must utilize Docusaurus v3 or a newer version, and include essential features such as dark mode, integrated search functionality, and content versioning.
*   All code examples and practical guidance for robotics must be based on ROS 2 LTS (e.g., Jazzy) for stability and long-term support.
*   All simulation examples must use a recent stable version of Isaac Sim (e.g., 2024.x).

*   The RAG chatbot will utilize a free Google Gemini LLM (e.g., gemini-1.5-flash) to adhere to free-tier constraints.
*   The Docusaurus site must be fully responsive, ensuring optimal user experience and display across both mobile and desktop devices.
*   The total size of the GitHub repository, including all code, documentation, and assets, must not exceed 2 GB.


## Non-Goals
*   Integration with physical robot hardware or control systems is explicitly out of scope.
*   The creation or embedding of video lectures or YouTube content is not part of this project.
*   No payment or subscription system for accessing content or features will be implemented.
*   Real-time collaborative editing functionalities for textbook content are excluded.
*   Development of an offline Progressive Web Application (PWA) is not a project goal.
*   The capability to export textbook content to PDF format is not included.
*   Implementation of a discussion forum or community interaction features is outside the project scope.
*   Translation features (e.g., Urdu translation) are explicitly excluded to maintain 100% free operation.
*   User authentication or account systems are excluded from the RAG chatbot functionality.


## Constitution Alignment Check
The project strongly aligns with the constitution's principles of:
*   **Knowledge Dissemination**: Creating an AI-native textbook fosters education and widespread understanding in a critical emerging field.
*   **Open Source Contribution**: Adherence to the MIT license promotes collaboration and accessibility of the codebase.
*   **AI-Driven Innovation**: Leveraging AI for RAG chatbot functionality exemplifies innovative application of AI for enhanced learning experiences.
*   **Structured Development**: The mandated use of Spec-Kit Plus ensures a systematic, transparent, and robust development process, aligning with best practices for project governance and maintainability.
*   **Free and Open Access**: The project exclusively uses free services and removes all paid features to ensure universal accessibility.


---

## RAG Chatbot Implementation Requirements

### Technical Architecture

#### Backend Requirements
*   **Framework**: The RAG chatbot backend MUST utilize FastAPI as the primary web framework, with all endpoints implemented as asynchronous functions (`async def`) to ensure optimal performance and scalability.
*   **No Authentication**: The RAG chatbot MUST be fully public with no authentication, login, signup, sessions, JWT, middleware, guards, or protected routes.
*   **Vector Storage**: Qdrant Cloud Free Tier MUST be used for vector embeddings storage and retrieval, containing embeddings of all textbook content for accurate semantic search and retrieval.
*   **Generative AI**: Google Gemini API MUST be leveraged for content generation and response formulation, properly integrating retrieved context from the vector store with the LLM's generative capabilities. The project MUST use only the free tier of Google Gemini API.

#### RAG Functionality
*   **Full Book Querying**: The chatbot MUST be able to answer questions based on the entire content of the textbook with high accuracy.
*   **Selected-Text-Only Mode**: The chatbot MUST support a selected-text-only RAG mode where user queries are contextualized based only on highlighted text sections rather than the entire textbook corpus. This functionality restricts answers strictly to user-selected chunks.
*   **Ingestion Pipeline**: An automated ingestion pipeline MUST be created to read all book Markdown content, chunk content appropriately, generate embeddings, and persist them to the vector store during build/deploy with error handling and validation.

### User Interface and Experience

#### Frontend Components
*   **Docusaurus Integration**: All interactive features (chatbot) MUST be implemented as modular Docusaurus/React components and seamlessly integrated into the site.
*   **Modern UI**: The chatbot UI MUST be modern, clean, and professional with improved message bubbles, responsiveness, and dark/light mode support.
*   **Loading States**: All interactive features MUST include appropriate loading states to indicate processing to the user.
*   **Fallback Mechanisms**: The system MUST include fallback mechanisms for when external services fail.

### Quality and Performance Requirements

#### Performance Standards
*   **Async Endpoints**: All backend endpoints MUST be implemented asynchronously using `async def` functions for all chat queries and content retrieval.
*   **Error Handling**: Comprehensive error handling MUST be implemented throughout the application with graceful degradation in response to service failures (e.g., LLM unavailability, vector store downtime).

#### Quality Gates
*   **RAG Accuracy**: The RAG system MUST achieve >90% accuracy on test queries, with retrieval accuracy exceeding 90% for test queries.
*   **Testing Suite**: A comprehensive test suite MUST be implemented to measure RAG accuracy, validating that the system retrieves relevant context and generates correct answers with acceptable precision and recall metrics.
*   **Reusable Skills**: Reusable skills and subagents MUST be created specifically for chatbot workflows, with complex operations like RAG retrieval encapsulated in modular, testable components.

### Success Criteria for RAG Implementation
*   The RAG chatbot correctly answers at least 90% of test questions based solely on textbook content
*   Selected-text-only mode functions with >85% accuracy when users highlight specific content sections
*   Ingestion pipeline processes all book content within 10 minutes during deployment
*   System maintains sub-2 second response times for 95% of requests under normal load
*   All backend endpoints respond asynchronously with appropriate error handling
*   The system gracefully handles service outages without crashing
*   Reusable skills/subagents demonstrate clear separation of concerns and testability
*   The chatbot is fully public with no authentication requirements
*   The project uses only the free Google Gemini API with no paid features