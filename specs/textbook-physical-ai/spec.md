## Intent
The intent is to deliver an AI-Native Textbook on Physical AI and Humanoid Robotics, integrated into a publicly deployed Docusaurus site. This project, a deliverable for the Panaversity Hackathon I, will include core features such as a RAG chatbot and will be developed using Spec-Kit Plus. Additionally, it will incorporate bonus features like user authentication with personalization, and on-demand Urdu translation.

## User Personas
*   **Students/Learners**: Individuals seeking to learn about Physical AI and Humanoid Robotics, ranging from beginners to advanced practitioners, who benefit from structured content, code examples, diagrams, exercises, and further reading.
*   **Researchers/Developers**: Professionals and enthusiasts looking for in-depth information, practical code implementations, and a reliable reference for Physical AI concepts.
*   **Global Audiences**: Users who require content in languages other than English, specifically Urdu, to facilitate their learning and research.
*   **Personalized Learners**: Users who prefer tailored educational content based on their existing software and hardware experience.

## Success Evals
*   The textbook structure comprises exactly 13 or more chapters, adhering precisely to the predefined weekly breakdown.
*   Each chapter within the textbook consistently integrates relevant code examples, explanatory diagrams, practical exercises, and suggestions for further reading.
*   The live Docusaurus site is publicly accessible and functional at the specified GitHub Pages URL: `<username>.github.io/<repo>`.
*   The embedded RAG chatbot demonstrates high accuracy, correctly answering at least 95% of a set of 20 predefined test questions based solely on the textbook's content.
*   The chatbot's functionality extends to accurately answering questions when users highlight and select specific text within the textbook as context.
*   The on-demand Urdu translation feature is operational across all chapters, providing translations with a latency not exceeding 3 seconds per request.
*   The "Personalize Content" button successfully modifies a minimum of 30% of the examples and/or code difficulty within a chapter, based on the user's stored profile.
*   The entire project's source code is licensed under the MIT License, exclusively utilizes free-tier cloud services, and contains no hard-coded secrets.
*   Full Spec-Kit Plus artifacts, including Architecture Decision Records (ADRs), Product Health Reports (PHRs), and historical records, are fully preserved within the project.
*   The completed project is submitted before the deadline of Sunday, November 30, 2025, 06:00 PM PKT.

## Constraints
*   The project must exclusively rely on free-tier services for all deployed components and infrastructure.
*   The Docusaurus site implementation must utilize Docusaurus v3 or a newer version, and include essential features such as dark mode, integrated search functionality, and content versioning.
*   All Urdu translations provided by the system must achieve a high level of linguistic quality, comparable to human translation, and avoid the characteristics of poor machine translation.
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

## Constitution Alignment Check
The project strongly aligns with the constitution's principles of:
*   **Knowledge Dissemination**: Creating an AI-native textbook fosters education and widespread understanding in a critical emerging field.
*   **Open Source Contribution**: Adherence to the MIT license promotes collaboration and accessibility of the codebase.
*   **AI-Driven Innovation**: Leveraging AI for content personalization and RAG chatbot functionality exemplifies innovative application of AI for enhanced learning experiences.
*   **Structured Development**: The mandated use of Spec-Kit Plus ensures a systematic, transparent, and robust development process, aligning with best practices for project governance and maintainability.
*   **Accessibility and Inclusivity**: Providing on-demand Urdu translation supports a broader, more inclusive global audience.
