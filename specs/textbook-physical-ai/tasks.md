---
description: "Task list for implementing the Physical AI & Humanoid Robotics Textbook"
---

# Tasks: Physical AI & Humanoid Robotics AI-Native Textbook

**Input**: `specs/textbook-physical-ai/plan.md`, `specs/textbook-physical-ai/spec.md`
**Prerequisites**: All design documents finalized.

## Format: `[ID] [P?] [US?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[USX]**: Which user story this task belongs to (e.g., US1, US2, US3)

---

## Phase 1: Constitution & Setup (Shared Infrastructure)

**Purpose**: Project initialization, repository setup, and finalizing core specifications.
**Success Criteria**: Docusaurus site runs locally; `constitution.md` and `spec.md` are ratified; GitHub repo is set up with protected branches.

- [ ] T001 Initialize Docusaurus project in the root directory.
- [ ] T002 Configure Docusaurus `docusaurus.config.ts` with project title, theme, and navigation.
- [ ] T003 [P] Finalize and ratify `constitution.md` in `.specify/memory/constitution.md`.
- [ ] T004 [P] Finalize and ratify `spec.md` in `specs/textbook-physical-ai/spec.md`.
- [ ] T005 [P] Setup GitHub repository with a `main` branch and branch protection rules.
- [ ] T006 [P] Configure Prettier and ESLint for code consistency in `package.json`.
- [ ] T007 [REUSABLE INTELLIGENCE CANDIDATE] Create `Subagent:SpecScanner` to read `spec.md` and generate a test-plan checklist.

**CHECKPOINT**: Project is set up, specifications are locked, and the base Docusaurus site is running.

---

## Phase 2: Core Book Writing & Content Structure

**Purpose**: Draft the core chapters of the textbook and establish the content structure in Docusaurus.
**Success Criteria**: At least 10 of 13+ chapters are drafted in Markdown and correctly structured.

- [ ] T008 Establish the book's chapter structure in `sidebars.ts`.
- [ ] T009 [P] Draft Chapter 1: Introduction to Physical AI in `docs/intro.md`.
- [ ] T010 [P] Draft Chapter 2: Foundations of Robotics in `docs/foundations-of-robotics.md`.
- [ ] T011 [P] Draft Chapter 3: Robot Kinematics and Dynamics in `docs/kinematics-dynamics.md`.
- [ ] T012 [P] Draft Chapter 4: Sensors and Perception in `docs/sensors-perception.md`.
- [ ] T013 [P] Draft Chapter 5: Control Systems for Robotics in `docs/control-systems.md`.
- [ ] T014 [P] Draft Chapter 6: Introduction to ROS 2 in `docs/ros2-intro.md`.
- [ ] T015 [P] Draft Chapter 7: Sim-to-Real with Isaac Sim in `docs/isaac-sim.md`.
- [ ] T016 [P] Draft Chapter 8: SLAM and Navigation in `docs/slam-navigation.md`.
- [ ] T017 [P] Draft Chapter 9: Manipulation and Grasping in `docs/manipulation-grasping.md`.
- [ ] T018 [P] Draft Chapter 10: Vision-Language-Action (VLA) Models in `docs/vla-models.md`.
- [ ] T019 [P] Draft Chapter 11: Humanoid Robotics Architectures in `docs/humanoid-architectures.md`.
- [ ] T020 [P] Draft Chapter 12: Ethics in Physical AI in `docs/ethics.md`.
- [ ] T021 [P] Draft Chapter 13: Future of Humanoid Robotics in `docs/future-humanoids.md`.
- [ ] T022 [REUSABLE INTELLIGENCE CANDIDATE] Implement `Skill:MarkdownChapterGenerator` to draft a chapter template from an outline for `docs/new-chapter-template.md`.
- [ ] T023 [REUSABLE INTELLIGENCE CANDIDATE] Implement `Skill:RoboticsCodeGenerator` to generate a sample ROS 2 publisher node in Python for `docs/ros2-examples/publisher.py`.

**CHECKPOINT**: Core textbook content is drafted and structured, ready for the RAG pipeline.

---

## Phase 3: RAG Backend Development

**Purpose**: Build and deploy the FastAPI backend, populate the vector database, and create a functional RAG pipeline.
**Success Criteria**: The `/rag` API endpoint returns structured answers from queries against the book content. Qdrant collection has over 1,000 vectors.

- [ ] T024 Initialize FastAPI project structure in `backend/`.
- [ ] T025 [P] Configure Neon Serverless Postgres and set up database connection in `backend/app/core/db.py`.
- [ ] T026 [P] Configure Qdrant Cloud connection in `backend/app/core/qdrant.py`.
- [ ] T027 [P] Implement data models for users and conversation history using SQLAlchemy in `backend/app/models/`.
- [ ] T028 [REUSABLE INTELLIGENCE CANDIDATE] Implement `Skill:QdrantIndexer`, a script to chunk, embed, and index all Markdown files from `docs/` into Qdrant in `scripts/ingest.py`.
- [ ] T029 Run the ingestion script to populate the Qdrant vector database.
- [ ] T030 Implement the RAG pipeline logic to retrieve context from Qdrant and generate answers using a local LLM in `backend/app/services/rag_service.py`.
- [ ] T031 Create the `/rag` API endpoint in `backend/app/api/rag.py` to handle user queries.
- [ ] T032 Implement "selected text only" logic in `backend/app/services/rag_service.py`.
- [ ] T033 Set up Docker for the backend application in `backend/Dockerfile`.
- [ ] T034 Deploy the FastAPI backend to a free-tier service (e.g., Vercel, Heroku).

**CHECKPOINT**: The RAG backend is live and capable of answering queries based on the textbook content.

---

## Phase 4: Frontend Integration & Chatbot UI

**Purpose**: Integrate the RAG chatbot into the Docusaurus site.
**Success Criteria**: The chatbot UI is functional, and users can receive answers from both full-book and "selected text" queries.

- [ ] T035 [P] Style the Docusaurus site using TailwindCSS for a modern, clean aesthetic in `src/css/custom.css`.
- [ ] T036 Create a new React component for the chatbot UI using ChatKit in `src/components/Chatbot/index.tsx`.
- [ ] T037 Implement state management for the chatbot (loading, messages, errors) in `src/components/Chatbot/index.tsx`.
- [ ] T038 Integrate the backend `/rag` API with the chatbot component to send queries and display responses.
- [ ] T039 Implement the frontend logic to capture selected text and send it to the backend for "selected text only" queries.
- [ ] T040 Embed the Chatbot component into the main layout of the Docusaurus site.

**CHECKPOINT**: The RAG chatbot is fully integrated and functional on the live website.

---

## Phase 5: Bonus Features Implementation

**Purpose**: Implement user authentication, content personalization, and Urdu translation.
**Success Criteria**: All bonus features are fully functional as per the `spec.md`.

### User Story 1: Authentication & Personalization Profile
- [ ] T041 [US1] [P] Set up Better-Auth for user authentication.
- [ ] T042 [US1] Implement FastAPI endpoints for user signup, login, and profile management in `backend/app/api/auth.py`.
- [ ] T043 [US1] Create a background questionnaire UI as part of the signup process in `src/pages/signup.tsx`.
- [ ] T044 [US1] Store user profile and questionnaire responses in the Neon database.
- [ ] T045 [US1] [P] Create login and signup pages in the Docusaurus site in `src/pages/login.tsx` and `src/pages/signup.tsx`.
- [ ] T046 [US1] Implement frontend logic to store auth tokens in local storage.

### User Story 2: Content Personalization
- [ ] T047 [US2] Create a backend service to generate contextual notes based on user profile in `backend/app/services/personalization_service.py`.
- [ ] T048 [US2] Create a "Personalize Content" button component in `src/components/PersonalizeButton/index.tsx`.
- [ ] T049 [US2] Implement logic to fetch and inject personalized notes into the chapter content when the button is clicked.

### User Story 3: Urdu Translation
- [ ] T050 [US3] [REUSABLE INTELLIGENCE CANDIDATE] Implement `Skill:UrduTranslator` to translate text using a local NLLB model in `backend/app/services/translation_service.py`.
- [ ] T051 [US3] Create the `/translate` API endpoint in `backend/app/api/translate.py`.
- [ ] T052 [US3] [P] Add a site-wide language toggle switch to the main navigation bar in `docusaurus.config.ts`.
- [ ] T053 [US3] Implement frontend logic to call the translation API for the current chapter and render the translated content.
- [ ] T054 [US3] Implement error handling for translation failures, reverting to English.

**CHECKPOINT**: All bonus features are implemented, tested end-to-end, and functional.

---

## Phase 6: Polish, Deployment & Finalization

**Purpose**: Finalize the project, perform end-to-end testing, and deploy the full application.
**Success Criteria**: The site is live at its public URL, fully responsive, and achieves a Lighthouse score over 90.

- [ ] T055 Conduct a full end-to-end test of all features using Playwright.
- [ ] T056 [P] Perform a comprehensive review of mobile responsiveness and fix any layout issues.
- [ ] T057 [P] Create and validate a "golden dataset" of 50+ Q&A pairs for RAG accuracy testing in `tests/rag_accuracy_test.py`.
- [ ] T058 [P] Optimize frontend assets and site performance to achieve a Lighthouse score > 90.
- [ ] T059 [P] Add MIT License to the repository in `LICENSE`.
- [ ] T060 Configure CI/CD with GitHub Actions to build and deploy the Docusaurus site to GitHub Pages.
- [ ] T061 Finalize all Spec-Kit Plus artifacts (`ADRs`, `PHRs`).
- [ ] T062 [REUSABLE INTELLIGENCE CANDIDATE] Create `Subagent:ADRGenerator` to draft an ADR for a key architectural decision.

**CHECKPOINT**: The project is feature-complete, polished, and deployed.

---

## Phase 7: Demo Video & Submission

**Purpose**: Create a compelling demo video and prepare the project for submission.
**Success Criteria**: A high-quality demo video is produced and the project is submitted.

- [ ] T063 [P] Write a script for the demo video, showcasing all key features.
- [ ] T064 Record the demo video, ensuring high-quality audio and video.
- [ ] T065 [P] Edit the demo video, adding titles and callouts.
- [ ] T066 Prepare the final submission for the hackathon.

---
## Dependencies & Execution Order

- **Phase 1** must be completed before any other phase can start.
- **Phase 2** is a prerequisite for **Phase 3**.
- **Phase 3** is a prerequisite for **Phase 4**.
- **Phase 4** is a prerequisite for **Phase 5**.
- **Phase 5** is a prerequisite for **Phase 6**.
- **Phase 6** is a prerequisite for **Phase 7**.

### Parallel Opportunities

- **Phase 2**: All chapter drafting tasks (T009-T021) can be done in parallel.
- **Phase 3**: Database setup (T025, T026) can be parallelized with initial FastAPI setup (T024).
- **Phase 5**: The three user stories (Auth, Personalization, Translation) can be developed in parallel by different team members after Phase 4 is complete.
- **Phase 6**: Testing, optimization, and documentation tasks (T056-T062) can largely be done in parallel.