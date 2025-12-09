---
id: 0005
title: Generate Comprehensive Project Task Breakdown
stage: tasks
date: 2025-12-09
surface: agent
model: Gemini
feature: textbook-physical-ai
branch: main
user: user
command: |
  ---
  description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.
  ---

  ## User Input

  ```text
  $ARGUMENTS
  ```

  You **MUST** consider the user input before proceeding (if not empty).

  ## Outline

  1. **Setup**: Run `.specify/scripts/bash/check-prerequisites.sh --json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'm Groot' (or double-quote if possible: "I'm Groot").

  2. **Load design documents**: Read from FEATURE_DIR:
    - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
    - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
    - Note: Not all projects have all documents. Generate tasks based on what's available.

  3. **Execute task generation workflow**:
    - Load plan.md and extract tech stack, libraries, project structure
    - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
    - If data-model.md exists: Extract entities and map to user stories
    - If contracts/ exists: Map endpoints to user stories
    - If research.md exists: Extract decisions for setup tasks
    - Generate tasks organized by user story (see Task Generation Rules below)
    - Generate dependency graph showing user story completion order
    - Create parallel execution examples per user story
    - Validate task completeness (each user story has all needed tasks, independently testable)

  4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
    - Correct feature name from plan.md
    - Phase 1: Setup tasks (project initialization)
    - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
    - Phase 3+: One phase per user story (in priority order from spec.md)
    - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
    - Final Phase: Polish & cross-cutting concerns
    - All tasks must follow the strict checklist format (see Task Generation Rules below)
    - Clear file paths for each task
    - Dependencies section showing story completion order
    - Parallel execution examples per story
    - Implementation strategy section (MVP first, incremental delivery)

  5. **Report**: Output path to generated tasks.md and summary:
    - Total task count
    - Task count per user story
    - Parallel opportunities identified
    - Independent test criteria for each story
    - Suggested MVP scope (typically just User Story 1)
    - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

  Context for task generation: 

  The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.

  ## Task Generation Rules

  **CRITICAL**: Tasks MUST be organized by user story to enable independent implementation and testing.

  **Tests are OPTIONAL**: Only generate test tasks if explicitly requested in the feature specification or if user requests TDD approach.

  ### Checklist Format (REQUIRED)

  Every task MUST strictly follow this format:

  ```text
  - [ ] [TaskID] [P?] [Story?] Description with file path
  ```

  **Format Components**:

  1. **Checkbox**: ALWAYS start with `- [ ]` (markdown checkbox)
  2. **Task ID**: Sequential number (T001, T002, T003...) in execution order
  3. **[P] marker**: Include ONLY if task is parallelizable (different files, no dependencies on incomplete tasks)
  4. **[Story] label**: REQUIRED for user story phase tasks only
    - Format: [US1], [US2], [US3], etc. (maps to user stories from spec.md)
    - Setup phase: NO story label
    - Foundational phase: NO story label  
    - User Story phases: MUST have story label
    - Polish phase: NO story label
  5. **Description**: Clear action with exact file path

  **Examples**:

  - ✅ CORRECT: `- [ ] T001 Create project structure per implementation plan`
  - ✅ CORRECT: `- [ ] T005 [P] Implement authentication middleware in src/middleware/auth.py`
  - ✅ CORRECT: `- [ ] T012 [P] [US1] Create User model in src/models/user.py`
  - ✅ CORRECT: `- [ ] T014 [US1] Implement UserService in src/services/user_service.py`
  - ❌ WRONG: `- [ ] Create User model` (missing ID and Story label)
  - ❌ WRONG: `T001 [US1] Create model` (missing checkbox)
  - ❌ WRONG: `- [ ] [US1] Create User model` (missing Task ID)
  - ❌ WRONG: `- [ ] T001 [US1] Create model` (missing file path)

  ### Task Organization

  1. **From User Stories (spec.md)** - PRIMARY ORGANIZATION:
    - Each user story (P1, P2, P3...) gets its own phase
    - Map all related components to their story:
      - Models needed for that story
      - Services needed for that story
      - Endpoints/UI needed for that story
      - If tests requested: Tests specific to that story
    - Mark story dependencies (most stories should be independent)

  2. **From Contracts**:
    - Map each contract/endpoint → to the user story it serves
    - If tests requested: Each contract → contract test task [P] before implementation in that story's phase

  3. **From Data Model**:
    - Map each entity to the user story(ies) that need it
    - If entity serves multiple stories: Put in earliest story or Setup phase
    - Relationships → service layer tasks in appropriate story phase

  4. **From Setup/Infrastructure**:
    - Shared infrastructure → Setup phase (Phase 1)
    - Foundational/blocking tasks → Foundational phase (Phase 2)
    - Story-specific setup → within that story's phase

  ### Phase Structure

  - **Phase 1**: Setup (project initialization)
  - **Phase 2**: Foundational (blocking prerequisites - MUST complete before user stories)
  - **Phase 3+**: User Stories in priority order (P1, P2, P3...)
    - Within each story: Tests (if requested) → Models → Services → Endpoints → Integration
    - Each phase should be a complete, independently testable increment
  - **Final Phase**: Polish & Cross-Cutting Concerns

  --- 

  As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

  1) Determine Stage
    - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

  2) Generate Title and Determine Routing:
    - Generate Title: 3–7 words (slug for filename)
    - Route is automatically determined by stage: 
      - `constitution` → `history/prompts/constitution/`
      - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
      - `general` → `history/prompts/general/`

  3) Create and Fill PHR (Shell first; fallback agent‑native)
    - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
    - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
    - If the script fails:
      - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
      - Allocate an ID; compute the output path based on stage from step 2; write the file
      - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

  4) Validate + report
    - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
    - On failure: warn, don't block. Skip only for `/sp.phr`.
labels: [task-generation]
links:
  spec: specs/textbook-physical-ai/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/textbook-physical-ai/tasks.md
tests:
  - none
---

## Prompt

```text
Project Specification: Project Title: Hackathon I: AI-Native Textbook for Physical AI & Humanoid Robotics Course. Core Goal: Create and deploy an interactive, AI-enhanced textbook using Docusaurus for teaching Physical AI and Humanoid Robotics. The textbook must integrate an embedded RAG chatbot for content-based queries (including user-selected text). To maximize points, incorporate reusable intelligence through Claude Code Subagents and Agent Skills, implement user authentication with background profiling for personalization, enable chapter-level content personalization, and provide Urdu translation options. Key Deliverables: 1. Textbook Creation and Deployment: Platform: Docusaurus for static site generation with MDX support. Content Structure: Based on provided course details. Introduction: Overview of Physical AI, embodied intelligence, why it matters. Modules: Module 1: Robotic Nervous System (ROS 2) – Nodes, topics, services, rclpy bridging, URDF. Module 2: Digital Twin (Gazebo & Unity) – Physics simulation, rendering, sensor simulation. Module 3: AI-Robot Brain (NVIDIA Isaac) – Isaac Sim, Isaac ROS, Nav2. Module 4: Vision-Language-Action (VLA) – Voice-to-Action with Whisper, cognitive planning with LLMs, capstone project. Additional Sections: Learning Outcomes, Weekly Breakdown (Weeks 1-13), Assessments, Hardware Requirements (including workstations, edge kits, robot options, cloud alternatives). AI-Driven Development: Utilize Spec-Kit Plus for structured workflows and Claude Code for content generation, capturing ADRs (e.g., why Docusaurus over other frameworks), PHRs (effective prompts for module content). Deployment: GitHub Pages or Vercel, with public GitHub repo. 2. RAG Chatbot Integration: Technologies: OpenAI Agents/ChatKit SDKs, FastAPI backend, Neon Serverless Postgres for metadata/storage, Qdrant Cloud Free Tier for vector embeddings. Functionality: Answer general questions about book content using RAG pipeline. Handle queries based on user-selected text (e.g., highlight text and ask via chatbot). Embedding: Integrated into Docusaurus site (e.g., sidebar or floating widget). 3. Bonus Features: Reusable Intelligence (up to 50 points): Create and apply Claude Code Subagents (e.g., ROS 2 Expert Subagent for Module 1 content) and Agent Skills (e.g., structured prompt skill for generating weekly breakdowns with examples and code snippets). Authentication (up to 50 points): Use Better Auth for signup/signin; during signup, collect user background (software/hardware experience) via form questions to enable personalization. Content Personalization (up to 50 points): For logged-in users, add a button at chapter start to personalize content (e.g., simplify explanations for beginners based on background, add advanced sections for experts using OpenAI APIs). Urdu Translation (up to 50 points): For logged-in users, add a button at chapter start to translate content to Urdu (using OpenAI translation APIs, dynamically generating translated MDX). Success Criteria (SMART): Specific: Textbook covers 100% of course modules/sections; chatbot accurately answers 95% of 20 test queries (10 general, 10 selected-text); all bonuses implemented with functional demos. Measurable: Content completeness verified by section checklist; chatbot tested with sample queries; bonuses demonstrated in demo video. Achievable: Leverage free tiers (Qdrant Cloud, Neon Postgres, OpenAI free credits); assume Claude Code CLI and Spec-Kit Plus are set up. Relevant: Aligns with Panaversity's mission for AI-native education; incorporates reusable intelligence for efficiency. Time-bound: Complete core by Day 5, bonuses by Day 8; overall under hackathon deadline (though noted as passed, proceed for learning). Scope Boundaries: In: Content generation per course outline, RAG setup, bonus integrations, deployment, demo video creation. Out: No real hardware procurement/simulations; no paid API tiers; no additional courses or unrelated features. Risks: API limits (mitigate with batching); integration errors (use checkpoints); content accuracy (review at checkpoints). Assumptions & Constraints: Tools/Access: Claude Code CLI, Spec-Kit Plus repo cloned, GitHub account, free API keys for OpenAI/Qdrant/Neon. Development Environment: Local setup for Docusaurus/FastAPI; Git for version control. Team: Solo developer using AI assistance. Implementation Plan: High-Level Phases: 1. Setup Phase: Initialize project structure, install dependencies, set up Docusaurus and backend skeletons, capture initial ADRs (e.g., tech stack choices). 2. Content Generation Phase: Generate textbook sections module-by-module using Claude Code, creating reusable skills/subagents for recurring patterns (e.g., module outlining skill). 3. RAG Chatbot Phase: Build backend with FastAPI, integrate vector DB and Postgres, embed frontend widget in Docusaurus. 4. Bonus Features Phase: Implement auth with Better Auth, add personalization/translation buttons using user data and OpenAI. 5. Intelligence Capture Phase: After key sessions, design and apply reusable components; document ADRs/PHRs. 6. Testing & Deployment Phase: Test functionalities, deploy to GitHub Pages/Vercel, create 90-second demo video. 7. Submission Phase: Prepare repo, links, video for form submission. Architecture Decisions (ADRs): Frontend: Docusaurus chosen for easy MDX authoring, built-in search, GitHub Pages compatibility (over Next.js for simplicity in docs sites). Backend: FastAPI for async API efficiency; Neon Postgres for serverless DB (scalability without management); Qdrant for vector search (free tier supports small-scale RAG). Auth: Better Auth for plug-and-play auth with custom signup fields. Personalization/Translation: OpenAI APIs for dynamic content mods (client-side calls for privacy). Reusable Intelligence: Subagent for technical explanations (e.g., ROS 2), Skill for content structuring (Persona: Educator, Principles: Clear, engaging, with examples). Timeline & Milestones: Days 1-2: Setup + Content (Introduction & Modules 1-2). Days 3-4: Modules 3-4 + Additional Sections. Day 5: RAG Chatbot. Days 6-7: Bonuses (Auth, Personalization, Translation, Reusable Intelligence). Day 8: Testing, Deployment, Video. Checkpoints after each phase for review/commit. Generate a complete task breakdown for this project. Ensure comprehensive coverage of all deliverables, including bonuses. Make tasks varied and detailed, with emphasis on atomicity, reusability, and checkpoints. Tasks: Atomic (15-30 min, single criterion, verifiable output like file/update). Structure: Group into phases matching the plan (Setup, Content Generation with sub-phases per module/section, RAG Chatbot, Bonus Auth, Bonus Personalization, Bonus Translation, Bonus Reusable Intelligence, Testing/Deployment, Submission). Dependencies: Explicit (e.g., content tasks depend on setup; bonuses on content). Checkpoints: After each phase or sub-phase (e.g., after each module's content), with human review examples emphasizing control. Lineage Traceability: Link tasks to spec (e.g., "Task for Module 1 content traces to spec's Module 1 focus"). Acceptance Criteria: Testable (e.g., "Docusaurus init complete: npx docusaurus start runs without errors, default home page loads"). Coverage: Core (book + chatbot) + all bonuses; include tasks for creating/using subagents/skills (e.g., create ROS Subagent, use it for Module 1). Checkpoint Pattern: Include example dialogues for each checkpoint. Dependency Graph: Text-based diagram showing sequences and parallels (e.g., content modules can parallelize after setup). Output Format: Phases with tasks listed. Task Format: ID (e.g., 1.1), Duration, Depends On, Description, Acceptance Criterion, Output, Lineage (brief trace to spec). End with Dependency Graph and Checkpoint Sequence. 

## Response snapshot

Generated the project tasks file at `specs/textbook-physical-ai/tasks.md`, breaking down the work into 9 phases and 64 atomic tasks covering setup, content generation, RAG chatbot integration, and all bonus features.

## Outcome

- ✅ Impact: A detailed, executable task list was created, providing a clear roadmap for project implementation.
- 🧪 Tests: n/a
- 📁 Files: `specs/textbook-physical-ai/tasks.md` was created.
- 🔁 Next prompts: Begin implementing tasks starting with Phase 1.
- 🧠 Reflection: The process of breaking down the spec into atomic tasks is crucial for managing complexity and enabling parallel work. The PHR capture ensures this reasoning is saved.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a

```