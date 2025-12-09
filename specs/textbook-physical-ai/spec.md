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