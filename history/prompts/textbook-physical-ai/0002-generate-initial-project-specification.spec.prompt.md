---
id: 0002
title: Generate Initial Project Specification
stage: spec
date: 2025-12-08
surface: agent
model: gemini
feature: textbook-physical-ai
branch: main
user: unknown
command: /sp.specify
labels: [spec, initialization]
links:
  spec: specs/textbook-physical-ai/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/textbook-physical-ai/spec.md
  - history/prompts/textbook-physical-ai/0002-generate-initial-project-specification.spec.prompt.md
tests:
  (none)
---

## Prompt

You are an expert Spec-Kit Plus practitioner following the official Specification Phase (Lesson 4) and your job is to generate a complete, crystal-clear, SMART-compliant specification file at specs/textbook-physical-ai/spec.md using the /sp.specify pattern for the Physical AI & Humanoid Robotics Full AI-Native Textbook + Deployed Docusaurus Site project as a Panaversity Hackathon I Deliverable with these core requirements that must be delivered: (1) complete textbook in Markdown using Docusaurus v3+, (2) publicly deployed on GitHub Pages, (3) embedded RAG chatbot using OpenAI Agents/ChatKit, FastAPI, Neon Serverless Postgres, Qdrant Cloud Free that answers from full book OR user-selected text only, (4) entire project built using Spec-Kit Plus with this being the first artifact, and these bonus features explicitly in scope that must reflect in spec: Better-Auth signup/signin + user background questionnaire for software/hardware experience, per-chapter "Personalize Content" button that adapts content based on user profile, per-chapter "اردو میں ترجمہ کریں" button for on-demand Urdu translation, with success evals that must be SMART and objectively verifiable including: textbook has 13+ chapters exactly matching the provided weekly breakdown, every chapter contains code examples, diagrams, exercises, and further reading, live site accessible at <username>.github.io/<repo>, RAG chatbot correctly answers ≥95% of 20 predefined test questions using only book content, chatbot accurately answers when user selects specific text only, Urdu translation button works on every chapter with latency < 3 seconds, personalization button modifies ≥30% of examples/code difficulty based on stored user profile, all source code is MIT licensed and runs on free tiers only with no hard-coded secrets, full Spec-Kit Plus artifacts preserved including ADRs, PHRs, and history/, project submitted before Sunday Nov 30 2025 06:00 PM PKT, and these non-negotiable constraints: only free-tier services allowed, Docusaurus v3+ with dark mode, search, and versioning, Urdu translations must be high-quality at human-level not poor MTL, fully responsive for mobile + desktop, repo size < 2 GB, with these non-goals explicitly excluded: physical robot hardware integration, video lectures or YouTube content, payment/subscription system, real-time collaborative editing, offline PWA, PDF export, discussion forum. Now run /sp.specify with feature name "textbook-physical-ai" and generate specs/textbook-physical-ai/spec.md containing ONLY these sections: Intent, User Personas, Success Evals, Constraints, Non-Goals, Constitution Alignment Check, written in clear professional English with zero implementation leakage so another developer must be able to build the entire textbook + site just from this spec, then after generating create the file and show full content.

## Response snapshot

I have created the `spec.md` file at `specs/textbook-physical-ai/spec.md` with the following content, which adheres to all your requirements.

# Specification: Physical AI & Humanoid Robotics Textbook

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

## Outcome

- ✅ Impact: Created the initial project specification file, establishing the foundation for the project.
- 🧪 Tests: (none)
- 📁 Files: Created `specs/textbook-physical-ai/spec.md` and this PHR.
- 🔁 Next prompts: The user can now proceed with clarifying the spec (`/sp.clarify`) or planning (`/sp.plan`).
- 🧠 Reflection: The creation of a detailed, SMART-compliant specification is a critical first step that aligns all stakeholders and reduces ambiguity for downstream development phases.

## Evaluation notes (flywheel)

- Failure modes observed: (none)
- Graders run and results (PASS/FAIL): (none)
- Prompt variant (if applicable): (none)
- Next experiment (smallest change to try): (none)