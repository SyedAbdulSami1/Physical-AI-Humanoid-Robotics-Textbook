# Gemini CLI Rules

This file is generated during init for the selected agent.

You are an expert AI assistant specializing in Spec-Driven Development (SDD). Your primary goal is to work with the architext to build products.

## Task context

**Your Surface:** You operate on a project level, providing guidance to users and executing development tasks via a defined set of tools.

**Your Success is Measured By:**
- All outputs strictly follow the user intent.
- Prompt History Records (PHRs) are created automatically and accurately for every user prompt.
- Architectural Decision Record (ADR) suggestions are made intelligently for significant decisions.
- All changes are small, testable, and reference code precisely.

## Core Guarantees (Product Promise)

- Record every user input verbatim in a Prompt History Record (PHR) after every user message. Do not truncate; preserve full multiline input.
- PHR routing (all under `history/prompts/`):
  - Constitution → `history/prompts/constitution/`
  - Feature-specific → `history/prompts/<feature-name>/`
  - General → `history/prompts/general/`
- ADR suggestions: when an architecturally significant decision is detected, suggest: "📋 Architectural decision detected: <brief>. Document? Run `/sp.adr <title>`." Never auto‑create ADRs; require user consent.

## Development Guidelines

### 1. Authoritative Source Mandate:
Agents MUST prioritize and use MCP tools and CLI commands for all information gathering and task execution. NEVER assume a solution from internal knowledge; all methods require external verification.

### 2. Execution Flow:
Treat MCP servers as first-class tools for discovery, verification, execution, and state capture. PREFER CLI interactions (running commands and capturing outputs) over manual file creation or reliance on internal knowledge.

### 3. Knowledge capture (PHR) for Every User Input.
After completing requests, you **MUST** create a PHR (Prompt History Record).

**When to create PHRs:**
- Implementation work (code changes, new features)
- Planning/architecture discussions
- Debugging sessions
- Spec/task/plan creation
- Multi-step workflows

**PHR Creation Process:**

1) Detect stage
   - One of: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate title
   - 3–7 words; create a slug for the filename.

2a) Resolve route (all under history/prompts/)
  - `constitution` → `history/prompts/constitution/`
  - Feature stages (spec, plan, tasks, red, green, refactor, explainer, misc) → `history/prompts/<feature-name>/` (requires feature context)
  - `general` → `history/prompts/general/`

3) Prefer agent‑native flow (no shell)
   - Read the PHR template from one of:
     - `.specify/templates/phr-template.prompt.md`
     - `templates/phr-template.prompt.md`
   - Allocate an ID (increment; on collision, increment again).
   - Compute output path based on stage:
     - Constitution → `history/prompts/constitution/<ID>-<slug>.constitution.prompt.md`
     - Feature → `history/prompts/<feature-name>/<ID>-<slug>.<stage>.prompt.md`
     - General → `history/prompts/general/<ID>-<slug>.general.prompt.md`
   - Fill ALL placeholders in YAML and body:
     - ID, TITLE, STAGE, DATE_ISO (YYYY‑MM‑DD), SURFACE="agent"
     - MODEL (best known), FEATURE (or "none"), BRANCH, USER
     - COMMAND (current command), LABELS (["topic1","topic2",...])
     - LINKS: SPEC/TICKET/ADR/PR (URLs or "null")
     - FILES_YAML: list created/modified files (one per line, " - ")
     - TESTS_YAML: list tests run/added (one per line, " - ")
     - PROMPT_TEXT: full user input (verbatim, not truncated)
     - RESPONSE_TEXT: key assistant output (concise but representative)
     - Any OUTCOME/EVALUATION fields required by the template
   - Write the completed file with agent file tools (WriteFile/Edit).
   - Confirm absolute path in output.

4) Use sp.phr command file if present
   - If `.**/commands/sp.phr.*` exists, follow its structure.
   - If it references shell but Shell is unavailable, still perform step 3 with agent‑native tools.

5) Shell fallback (only if step 3 is unavailable or fails, and Shell is permitted)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Then open/patch the created file to ensure all placeholders are filled and prompt/response are embedded.

6) Routing (automatic, all under history/prompts/)
   - Constitution → `history/prompts/constitution/`
   - Feature stages → `history/prompts/<feature-name>/` (auto-detected from branch or explicit feature context)
   - General → `history/prompts/general/`

7) Post‑creation validations (must pass)
   - No unresolved placeholders (e.g., `{{THIS}}`, `[THAT]`).
   - Title, stage, and dates match front‑matter.
   - PROMPT_TEXT is complete (not truncated).
   - File exists at the expected path and is readable.
   - Path matches route.

8) Report
   - Print: ID, path, stage, title.
   - On any failure: warn but do not block the main command.
   - Skip PHR only for `/sp.phr` itself.

### 4. Explicit ADR suggestions
- When significant architectural decisions are made (typically during `/sp.plan` and sometimes `/sp.tasks`), run the three‑part test and suggest documenting with:
  "📋 Architectural decision detected: <brief> — Document reasoning and tradeoffs? Run `/sp.adr <decision-title>`"
- Wait for user consent; never auto‑create the ADR.

### 5. Human as Tool Strategy
You are not expected to solve every problem autonomously. You MUST invoke the user for input when you encounter situations that require human judgment. Treat the user as a specialized tool for clarification and decision-making.

**Invocation Triggers:**
1.  **Ambiguous Requirements:** When user intent is unclear, ask 2-3 targeted clarifying questions before proceeding.
2.  **Unforeseen Dependencies:** When discovering dependencies not mentioned in the spec, surface them and ask for prioritization.
3.  **Architectural Uncertainty:** When multiple valid approaches exist with significant tradeoffs, present options and get user's preference.
4.  **Completion Checkpoint:** After completing major milestones, summarize what was done and confirm next steps. 

## Default policies (must follow)
- Clarify and plan first - keep business understanding separate from technical plan and carefully architect and implement.
- Do not invent APIs, data, or contracts; ask targeted clarifiers if missing.
- Never hardcode secrets or tokens; use `.env` and docs.
- Prefer the smallest viable diff; do not refactor unrelated code.
- Cite existing code with code references (start:end:path); propose new code in fenced blocks.
- Keep reasoning private; output only decisions, artifacts, and justifications.

### Execution contract for every request
1) Confirm surface and success criteria (one sentence).
2) List constraints, invariants, non‑goals.
3) Produce the artifact with acceptance checks inlined (checkboxes or tests where applicable).
4) Add follow‑ups and risks (max 3 bullets).
5) Create PHR in appropriate subdirectory under `history/prompts/` (constitution, feature-name, or general).
6) If plan/tasks identified decisions that meet significance, surface ADR suggestion text as described above.

### Minimum acceptance criteria
- Clear, testable acceptance criteria included
- Explicit error paths and constraints stated
- Smallest viable change; no unrelated edits
- Code references to modified/inspected files where relevant

## Architect Guidelines (for planning)

Instructions: As an expert architect, generate a detailed architectural plan for [Project Name]. Address each of the following thoroughly.

1. Scope and Dependencies:
   - In Scope: boundaries and key features.
   - Out of Scope: explicitly excluded items.
   - External Dependencies: systems/services/teams and ownership.

2. Key Decisions and Rationale:
   - Options Considered, Trade-offs, Rationale.
   - Principles: measurable, reversible where possible, smallest viable change.

3. Interfaces and API Contracts:
   - Public APIs: Inputs, Outputs, Errors.
   - Versioning Strategy.
   - Idempotency, Timeouts, Retries.
   - Error Taxonomy with status codes.

4. Non-Functional Requirements (NFRs) and Budgets:
   - Performance: p95 latency, throughput, resource caps.
   - Reliability: SLOs, error budgets, degradation strategy.
   - Security: AuthN/AuthZ, data handling, secrets, auditing.
   - Cost: unit economics.

5. Data Management and Migration:
   - Source of Truth, Schema Evolution, Migration and Rollback, Data Retention.

6. Operational Readiness:
   - Observability: logs, metrics, traces.
   - Alerting: thresholds and on-call owners.
   - Runbooks for common tasks.
   - Deployment and Rollback strategies.
   - Feature Flags and compatibility.

7. Risk Analysis and Mitigation:
   - Top 3 Risks, blast radius, kill switches/guardrails.

8. Evaluation and Validation:
   - Definition of Done (tests, scans).
   - Output Validation for format/requirements/safety.

9. Architectural Decision Record (ADR):
   - For each significant decision, create an ADR and link it.

### Architecture Decision Records (ADR) - Intelligent Suggestion

After design/architecture work, test for ADR significance:

- Impact: long-term consequences? (e.g., framework, data model, API, security, platform)
- Alternatives: multiple viable options considered?
- Scope: cross‑cutting and influences system design?

If ALL true, suggest:
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`

Wait for consent; never auto-create ADRs. Group related decisions (stacks, authentication, deployment) into one ADR when appropriate.

## Basic Project Structure

- `.specify/memory/constitution.md` — Project principles
- `specs/<feature>/spec.md` — Feature requirements
- `specs/<feature>/plan.md` — Architecture decisions
- `specs/<feature>/tasks.md` — Testable tasks with cases
- `history/prompts/` — Prompt History Records
- `history/adr/` — Architecture Decision Records
- `.specify/` — SpecKit Plus templates and scripts

## Code Standards
See `.specify/memory/constitution.md` for code quality, testing, performance, security, and architecture principles.

---
# Physical AI & Humanoid Robotics Textbook Content

## Course Overview
The course is titled **Physical AI & Humanoid Robotics** with the central focus and theme being **AI Systems in the Physical World and Embodied Intelligence**. The primary goal is bridging the gap between the digital brain and the physical body so that students can apply their existing AI knowledge to control humanoid robots in both simulated and real-world environments. This is a capstone quarter that introduces Physical AI — AI systems that operate in reality and understand physical laws — using ROS 2, Gazebo, and NVIDIA Isaac to design, simulate, and deploy humanoid robots capable of natural human interactions.

## Core Modules
The textbook must contain the following four core modules:

-   **Module 1: The Robotic Nervous System (ROS 2)**: Focuses on middleware for robot control, covering ROS 2 nodes, topics, services, bridging Python agents to ROS controllers using rclpy, and understanding URDF (Unified Robot Description Format) for humanoid robots.
-   **Module 2: The Digital Twin (Gazebo & Unity)**: Focuses on physics simulation and environment building, including simulating physics, gravity, and collisions in Gazebo, high-fidelity rendering and human-robot interaction in Unity, and simulating sensors such as LiDAR, depth cameras, and IMUs.
-   **Module 3: The AI-Robot Brain (NVIDIA Isaac™)**: Focuses on advanced perception and training, covering NVIDIA Isaac Sim for photorealistic simulation and synthetic data generation, Isaac ROS for hardware-accelerated VSLAM and navigation, and Nav2 for path planning in bipedal humanoid movement.
-   **Module 4: Vision-Language-Action (VLA)**: Focuses on the convergence of LLMs and robotics, including voice-to-action using OpenAI Whisper for voice commands, cognitive planning where LLMs translate natural language commands such as “Clean the room” into sequences of ROS 2 actions, and a final capstone project called **The Autonomous Humanoid** in which a simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it.

## Why Physical AI Matters
The textbook must also explain why Physical AI matters: humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from human environments, marking the transition from digital-only AI to true embodied intelligence that operates in physical space.

## Learning Outcomes
The learning outcomes that must be clearly stated are:
-   Understand Physical AI principles and embodied intelligence.
-   Master ROS 2 for robotic control.
-   Simulate robots with Gazebo and Unity.
-   Develop with the NVIDIA Isaac AI robot platform.
-   Design humanoid robots for natural interactions.
-   Integrate GPT models for conversational robotics.

## Weekly Breakdown (Weeks 1–13)
-   **Weeks 1–2: Introduction to Physical AI**: including foundations of embodied intelligence, the shift from digital AI to robots that understand physical laws, the humanoid robotics landscape, and sensor systems such as LiDAR, cameras, IMUs, and force/torque sensors.
-   **Weeks 3–5: ROS 2 Fundamentals**: including architecture, core concepts, nodes, topics, services, actions, building ROS 2 packages with Python, launch files, and parameter management.
-   **Weeks 6–7: Robot Simulation with Gazebo**: including environment setup, URDF and SDF formats, physics and sensor simulation, and an introduction to Unity for robot visualization.
-   **Weeks 8–10: The NVIDIA Isaac Platform**: including Isaac SDK and Isaac Sim, AI-powered perception and manipulation, reinforcement learning for robot control, and sim-to-real transfer techniques.
-   **Weeks 11–12: Humanoid Robot Development**: including kinematics, dynamics, bipedal locomotion, balance control, manipulation, grasping, and natural human-robot interaction design.
-   **Week 13: Conversational Robotics**: including integrating GPT models, speech recognition, natural language understanding, and multi-modal interaction using speech, gesture, and vision.

## Assessments
Assessments must be listed as:
-   ROS 2 package development project.
-   Gazebo simulation implementation.
-   Isaac-based perception pipeline.
-   Final Capstone of a simulated humanoid robot with conversational AI.

## Hardware Requirements
The entire Hardware Requirements section must be included without omitting anything: the course is technically demanding because it combines physics simulation, visual perception, and generative AI.

-   **“Digital Twin” Workstation**: Needs an NVIDIA RTX 4070 Ti (12 GB VRAM) or higher (ideally RTX 3090/4090 with 24 GB), Intel Core i7 13th Gen+ or AMD Ryzen 9, 64 GB DDR5 RAM (32 GB minimum), and Ubuntu 22.04 LTS.
-   **“Physical AI” Edge Kit**: Consists of NVIDIA Jetson Orin Nano or NX as the brain, Intel RealSense D435i/D455 for vision, a USB IMU, and a USB microphone/speaker array such as ReSpeaker.
-   **Three robot lab options are described**:
    -   Budget/proxy approach using Unitree Go2 Edu.
    -   Miniature humanoid approach using Unitree G1, Robotis OP3 or budget Hiwonder TonyPi Pro.
    -   Premium approach using Unitree G1 humanoid.
-   **Additional Information**: The architecture summary table, cloud-native “Ether” lab option with AWS g5/g6 instances, cost calculations, and the **Economy Jetson Student Kit** (Jetson Orin Nano Super Dev Kit $249 + RealSense D435i $349 + ReSpeaker $69 + misc $30 = ~$700 total) must all be present along with the latency trap explanation and the final solution of training in the cloud and flashing to local Jetson.


Here are **6 perfect, ready-to-paste English prompts** for **Cloud Code CLI / Gemini CLI / Claude Code** (works 100% even with token limits). Each prompt generates one clean chunk — run them one by one in order:

**Chunk 1 – Top-level pages**  
```
/sp.implement Generate full rich content only for the following top-level pages (nothing else):  1. docs/why-physical-ai-matters.md  2. docs/learning-outcomes.md  3. docs/weekly-breakdown.md (include all Weeks 1–13 exactly as given)  4. docs/assessments.md  5. docs/introduction.md  6. docs/hardware-requirements.md  Each page 900–1600 words, professional university tone, proper headings, tables, callouts, diagrams where needed, zero placeholders. Write all 6 files in parallel and at the end show only a table with file path + word count + status.
```

**Chunk 2 – Module 1 (ROS 2)**  
```
/sp.implement Generate full rich content only for Module 1: The Robotic Nervous System (ROS 2) — exactly these three sections:  
1. docs/module-1/ros2-nodes-topics-services.md  
2. docs/module-1/bridging-python-agents-to-ros2.md  
3. docs/module-1/urdf-for-humanoids.md  
Each section 1400–2200 words, minimum 6 real working tested Python/rclpy code examples, Mermaid diagrams, step-by-step labs, common pitfalls + fixes, student exercises with hidden solutions, official ROS 2 links, zero placeholders. Write all three in parallel and show table at the end.
```

**Chunk 3 – Module 2 (Gazebo & Unity)**  
```
/sp.implement Generate full rich content only for Module 2: The Digital Twin (Gazebo & Unity) — exactly these three sections:  
1. docs/module-2/simulating-physics-in-gazebo.md  
2. docs/module-2/high-fidelity-unity-rendering.md  
3. docs/module-2/simulating-sensors-lidar-depth-imu.md  
Each section 1400–2200 words, real working code + launch files, Gazebo/Unity screenshots references, Mermaid diagrams, labs, exercises with solutions, common errors, zero placeholders. Write all three in parallel and show table.
```

**Chunk 4 – Module 3 (NVIDIA Isaac)**  
```
/sp.implement Generate full rich content only for Module 3: The AI-Robot Brain (NVIDIA Isaac) — exactly these three sections:  
1. docs/module-3/isaac-sim-synthetic-data.md  
2. docs/module-3/isaac-ros-vslam-navigation.md  
3. docs/module-3/nav2-bipedal-path-planning.md  
Each section 1400–2200 words, real Isaac Sim + Isaac ROS examples, code snippets, synthetic data generation labs, VSLAM/ Nav2 tutorials, exercises, diagrams, zero placeholders. Write all three in parallel and show table.
```

**Chunk 5 – Module 4 Part 1 (VLA basics)**  
```
/sp.implement Generate full rich content only for Module 4 Vision-Language-Action (VLA) — first two sections:  
1. docs/module-4/voice-to-action-whisper.md  
2. docs/module-4/cognitive-planning-llm-to-ros2.md  
Each 1400–2200 words, real working Whisper + LLM + ROS 2 integration code, full examples of “Clean the room” → ROS actions, diagrams, step-by-step labs, exercises, zero placeholders. Write both in parallel and show table.
```

**Chunk 6 – Module 4 Capstone + Final Polish**  
```
/sp.implement Generate the final capstone section + polish everything:  
1. docs/module-4/capstone-autonomous-humanoid.md (2000–3000 words, complete end-to-end project with voice command → perception → planning → manipulation, full code repo structure, video demo instructions, grading rubric)  
2. Then run a final pass on the entire docs/ folder: fix any remaining placeholders, add consistent navigation hints, improve all Mermaid diagrams, ensure every code block is tested and runnable. At the end show a complete table of all 20+ files with final word counts and confirm the textbook is 100% ready for publication.
```

/sp.constitution Create a comprehensive project constitution for the Physical AI & Humanoid Robotics AI-native textbook built with Docusaurus, deployed on GitHub Pages, with integrated RAG chatbot (FastAPI + Neon Postgres + Qdrant Cloud), Better-Auth signup/login including user background questionnaire for personalization, per-chapter Personalize button, and per-chapter Urdu translation button; enforce strict standards including university-level technical depth in all content, real working runnable code examples with explanations, Mermaid diagrams for concepts, step-by-step labs with terminal commands, student exercises with hidden solutions, professional encouraging tone, accessibility compliance, mobile responsiveness, zero placeholders, full test coverage where applicable, modular reusable components, maximum creation of reusable intelligence (skills/subagents) for bonus points, adherence to APA citation style if needed, high-quality Markdown formatting with proper headings tables callouts and frontmatter, consistent navigation, and optimization for fast loading; the constitution must define non-negotiable quality gates that every specification, plan, task, and implementation must respect, structured as clear articles with rationale, and output the complete .specify/memory/constitution.md file while confirming all principles will be enforced in downstream phases. kindly add this requirment in all .md file.

Yeh raha **4 powerful /sp.implement prompts** Gemini CLI ke liye. Har prompt ek major chunk complete karega taake pura bacha hua kaam (RAG Chatbot UI + Backend + Bonus Features + Deployment) sirf 4 prompts mein ho jaye. Har prompt safe hai – existing .md files sirf append karega, kuch delete nahi karega.

### Prompt 1: RAG Chatbot Backend (FastAPI + Neon + Qdrant + Ingestion)

```
/model gemini-2.5-flash
/sp.implement Focus only on creating the complete RAG chatbot backend without modifying or deleting any existing .md files (only append if needed). Generate the full FastAPI backend in app/ folder: main.py, routers/chat.py, routers/ingest.py, models.py, database.py (Neon Serverless Postgres setup with async SQLAlchemy), qdrant_client integration for Qdrant Cloud Free Tier, ingestion pipeline that loads all existing Markdown content from docs/ into Qdrant (chunking, embeddings using OpenAI), OpenAI/ChatKit SDK for generation, full support for normal queries and selected-text-only mode (answers restricted to provided chunks), async endpoints, proper error handling, environment variables setup; include requirements.txt and .env.example; provide step-by-step terminal commands to set up Neon, Qdrant Cloud, run ingestion, and test endpoints with curl examples; fix any import errors by adding __init__.py and using absolute imports; at the end show a table of all new/modified files with path and status.
```

### Prompt 2: RAG Chatbot Frontend UI + Embed in Docusaurus

```
/model gemini-2.5-flash
/sp.implement Focus only on creating the RAG chatbot frontend UI and embedding it in the Docusaurus book without deleting any existing content. Generate React components in src/theme/Chatbot/ (Chatbot.js, ChatMessage.js, SelectedTextProvider.js etc.), integrate with FastAPI backend (/chat endpoint), support normal chat and selected-text-only mode (capture user selection, send to backend), beautiful UI with message history, loading states, markdown rendering for responses, mobile responsive; add chatbot toggle button in navbar or fixed position; update docusaurus.config.js if needed for theme; provide usage instructions and testing steps; at the end show a table of all new files created with path and status.
```

### Prompt 3: Bonus Features – Auth, Personalize & Urdu Translation

```
/model gemini-2.5-flash
/sp.implement Implement all bonus features without deleting any existing content: integrate Better-Auth for signup/login with background questionnaire (software/hardware experience) stored in Neon Postgres; create auth components in src/theme/Auth/; add per-chapter "Personalize" button that fetches user profile and adapts content (e.g., show/hide advanced sections or change explanations); add per-chapter "Translate to Urdu" button that translates current page content using reliable method (Google Translate API or LibreTranslate) with loading state and fallback to English; create reusable Docusaurus components for both buttons; update relevant pages to include buttons; create reusable skills/subagents for personalization and translation logic; provide setup instructions for Better-Auth and translation service; at the end show a table of all new/modified files with path and status.
```

### Prompt 4: Final Testing, Deployment & Demo Prep

```
/model gemini-2.5-flash
/sp.implement Final phase: perform full testing of RAG chatbot (accuracy >90%, selected-text mode), auth flow, personalization, Urdu translation; fix any remaining bugs; add deployment configuration for Vercel/GitHub Pages with backend (use serverless or render.com for FastAPI); generate deployment scripts and .vercelignore if needed; create a short demo video script (<90 seconds) highlighting book content, chatbot, auth, personalize, Urdu translation; append final status summary to the end of plan.md and tasks.md confirming all deliverables complete; run final validation and at the end show a comprehensive table of the entire project status including backend running, frontend integrated, bonus features working, and deployment ready.
```

**Sequence mein chalao:**

1. Prompt 1 → Backend ready
2. Prompt 2 → Chatbot UI in book
3. Prompt 3 → Bonus features (max points)
4. Prompt 4 → Testing + Deployment + Demo

Bas yeh 4 prompts daal do ek ek karke – pura project 100% hackathon-ready ho jayega!  
Abhi Prompt 1 se shuru karo. Main yahin hun har step ke liye. 🚀

Yeh raha woh final phase ka prompt 3 chhote, manageable prompts mein divide kiya gaya hai taake Gemini CLI ek ek karke comfortably complete kar sake. Har prompt safe hai – existing content sirf append karega, delete nahi.

### Prompt 1: Testing & Bug Fixes

```
/model gemini-2.5-flash
/sp.implement Focus only on full testing and bug fixing without deleting any content: thoroughly test the RAG chatbot for accuracy >90% on book content queries and correct selected-text-only mode (answers strictly from selected chunks); test Better-Auth signup/login flow with questionnaire storage in Neon Postgres; test per-chapter Personalize button (content adapts correctly based on user background); test per-chapter Urdu translation button (accurate translation, loading states, fallback); identify and fix any remaining bugs in backend (FastAPI endpoints), frontend components, or integration; provide detailed test results with examples of successful queries and fixes applied; append a "Testing Results & Bug Fixes" section at the end of plan.md summarizing what was tested and resolved; at the end show a table of tested features with status (pass/fail/fixed).
```

### Prompt 2: Deployment Configuration

```
/model gemini-2.5-flash
/sp.implement Focus only on deployment setup without deleting any content: add full deployment configuration for the complete project (Docusaurus frontend + FastAPI backend); configure for Vercel (preferred) or GitHub Pages with backend hosted on Render.com or similar serverless platform; generate vercel.json or necessary config files, .vercelignore, deployment scripts (e.g., build commands, environment variables setup for Neon, Qdrant, OpenAI keys); include step-by-step instructions for deploying both frontend and backend; test local build with production settings; append a "Deployment Configuration" section at the end of plan.md with all commands and configs; at the end show a table of deployment-related files created/modified with path and status.
```

### Prompt 3: Demo Video Script + Final Status Summary

```
/model gemini-2.5-flash
/sp.implement Final step: create a concise demo video script (<90 seconds) that highlights navigating the book content, using the RAG chatbot (normal + selected-text mode), signing up with questionnaire, clicking Personalize button to see adapted content, clicking Urdu translation button; include exact narration lines, what to show on screen, and timing; perform final validation of all features; append a "Final Project Status Summary" section at the end of both plan.md and tasks.md confirming all base deliverables and bonus features are complete and working; append the full demo video script to the end of plan.md; run final project validation and at the end show a comprehensive table confirming: backend running, frontend integrated, RAG chatbot working, auth + personalization active, Urdu translation functional, deployment ready, and overall hackathon submission complete.
```

**Sequence:**

1. Prompt 1 daalo → Testing + fixes complete
2. Prompt 2 daalo → Deployment ready
3. Prompt 3 daalo → Demo script + final confirmation

Bas yeh 3 prompts daal do ek ek karke – pura project 100% finished, tested, deployable aur submission-ready ho jayega!

Abhi Prompt 1 se shuru karo. Main yahin hun last step tak. 🚀 Good luck bhai, top submission confirm!