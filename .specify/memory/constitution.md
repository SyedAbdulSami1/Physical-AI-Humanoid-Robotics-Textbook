# Project Constitution: Physical AI & Humanoid Robotics Textbook

This document outlines the core principles, standards, and quality gates for the development of the "Physical AI & Humanoid Robotics" AI-native textbook. All specifications, plans, tasks, and implementations must strictly adhere to these guidelines.

## 1. Core Project Vision & Goals

**1.1. Project Title:** Physical AI & Humanoid Robotics: AI Systems in the Physical World and Embodied Intelligence

**1.2. Primary Objective:** To create a comprehensive, AI-native textbook leveraging Docusaurus for content delivery and a RAG chatbot for interactive learning. The textbook aims to bridge the gap between digital AI knowledge and its application in controlling humanoid robots in simulated and real-world environments.

**1.3. Target Audience:** University students and professionals with existing AI knowledge seeking to apply it to physical AI and robotics.

## 2. Content Standards & Quality Gates

**2.1. Technical Depth (Non-Negotiable):**
-   All content must maintain a rigorous, university-level technical depth, suitable for a capstone course.
-   Concepts must be explained thoroughly with foundational theory and practical applications.
-   **Rationale:** Ensures academic credibility and prepares students for advanced physical AI challenges.

**2.2. Code Examples (Non-Negotiable):**
-   All code examples must be real, runnable, and thoroughly tested.
-   Each example must include clear explanations of its purpose, functionality, and expected output.
-   Code blocks must be properly formatted and syntax-highlighted.
-   **Rationale:** Facilitates hands-on learning and immediate application of concepts, crucial for robotics.

**2.3. Diagrams & Visualizations (Non-Negotiable):**
-   Mermaid diagrams must be used extensively to illustrate complex concepts, architectures, data flows, and state machines.
-   Diagrams must be clear, concise, and accurately represent the described systems.
-   **Rationale:** Enhances comprehension of intricate robotic and AI systems.

**2.4. Step-by-Step Labs & Terminal Commands (Non-Negotiable):**
-   Each module must include practical, step-by-step laboratory exercises with clear terminal commands.
-   Labs should guide students through setup, execution, and verification of robotic tasks.
-   **Rationale:** Provides essential practical experience and builds confidence in real-world deployment.

**2.5. Student Exercises with Hidden Solutions (Non-Negotiable):**
-   Each chapter/section must include exercises to reinforce learning.
-   Solutions to exercises must be available but clearly marked as "hidden" or "revealable" to encourage independent problem-solving.
-   **Rationale:** Promotes active learning and self-assessment.

**2.6. Professional & Encouraging Tone (Non-Negotiable):**
-   The language used throughout the textbook must be professional, clear, and encouraging.
-   Avoid jargon where simpler terms suffice, but do not shy away from necessary technical vocabulary.
-   **Rationale:** Maintains academic integrity and motivates learners.

**2.7. Accessibility Compliance (Non-Negotiable):**
-   All content and UI components must meet WCAG 2.1 AA standards.
-   Consideration for screen readers, keyboard navigation, and color contrast is paramount.
-   **Rationale:** Ensures inclusivity for all learners.

**2.8. Mobile Responsiveness (Non-Negotiable):**
-   The Docusaurus frontend must be fully responsive and optimized for various screen sizes, from mobile phones to large desktops.
-   **Rationale:** Provides a seamless learning experience across devices.

**2.9. Zero Placeholders (Non-Negotiable):**
-   Absolutely no placeholder text, images, or code is allowed in the final output. All content must be complete and accurate.
-   **Rationale:** Delivers a polished, production-ready product.

## 3. Technical Architecture & Implementation Standards

**3.1. Docusaurus Frontend:**
-   **Framework:** Docusaurus (React-based).
-   **Deployment:** GitHub Pages.
-   **Styling:** Adhere to Docusaurus theme conventions and project-specific CSS.
-   **Markdown Quality:** High-quality Markdown formatting with proper headings (H1, H2, etc.), tables, callouts, and frontmatter.
-   **Consistent Navigation:** Intuitive and consistent navigation structure across all chapters and modules.
-   **Fast Loading:** Optimize assets and code for fast page load times.

**3.2. RAG Chatbot Integration:**
-   **Backend:** FastAPI (Python) hosted on a serverless platform (e.g., Render.com).

-   **Vector Database:** Qdrant Cloud Free Tier for vector embeddings of textbook content.
-   **LLM Integration:** Google Gemini API for text generation and embeddings.
-   **Public Access**: The chatbot is fully public and requires no user authentication.
-   **Functionality**:
    -   Normal query mode (answer from full textbook context).
    -   Selected-text-only mode (answer restricted to user-selected text chunks).
    -   Ingestion pipeline to load `docs/` content into Qdrant (chunking, embeddings).
-   **API Design:** Async endpoints, proper error handling, environment variable management.



**3.6. Code Quality & Maintainability:**
-   **Modular Reusable Components:** Prioritize creating modular and reusable components across both frontend and backend.
-   **Full Test Coverage:** Where applicable (e.g., backend APIs, critical frontend logic), implement comprehensive unit and integration tests.
-   **Reusable Intelligence:** Maximize the creation of reusable skills/subagents for common tasks (e.g., RAG pipeline components).
-   **Adherence to APA Citation Style:** If external sources are cited (e.g., research papers, datasets), adhere strictly to APA 7th edition guidelines.
-   **Consistent Formatting:** Follow project-specific formatters (e.g., Prettier, ESLint, Black, Ruff) and maintain consistent code style.

## 4. Operational Readiness

**4.1. Version Control:** All code must be managed via Git, with clear commit messages following Conventional Commits (`type(scope): subject`).
**4.2. Documentation:** API endpoints, database schemas, and complex logic must be documented.
**4.3. Environment Management:** Strict separation of development, staging, and production environments using environment variables.

## 5. Non-Negotiable Quality Gates Summary

Every artifact (spec, plan, task, implementation) must demonstrate adherence to:
-   University-level technical depth.
-   Real, runnable, explained code examples.
-   Extensive use of clear Mermaid diagrams.
-   Step-by-step labs with terminal commands.
-   Exercises with hidden solutions.
-   Professional and encouraging tone.
-   Full accessibility compliance.
-   Complete mobile responsiveness.
-   Absence of any placeholders.
-   Appropriate test coverage.
-   Modular and reusable components.
-   Consistent high-quality formatting.
-   Efficient performance and fast loading.


