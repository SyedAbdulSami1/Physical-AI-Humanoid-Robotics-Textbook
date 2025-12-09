---
id: 0007
title: Audit project artifacts against golden reference
stage: spec
date: 2025-12-09
surface: agent
model: gemini-1.5-pro
feature: textbook-physical-ai
branch: null
user: User
command: |
  ---
  description: Identify underspecified areas in the current feature spec by asking up to 5 highly targeted clarification questions and encoding answers back into the spec.
  ---

  ## User Input

  ```text
  $ARGUMENTS
  ```

  You **MUST** consider the user input before proceeding (if not empty).

  ## Outline

  Goal: Detect and reduce ambiguity or missing decision points in the active feature specification and record the clarifications directly in the spec file.

  Note: This clarification workflow is expected to run (and be completed) BEFORE invoking `/sp.plan`. If the user explicitly states they are skipping clarification (e.g., exploratory spike), you may proceed, but must warn that downstream rework risk increases.

  Execution steps:

  1. Run `.specify/scripts/bash/check-prerequisites.sh --json --paths-only` from repo root **once** (combined `--json --paths-only` mode / `-Json -PathsOnly`). Parse minimal JSON payload fields:
     - `FEATURE_DIR`
     - `FEATURE_SPEC`
     - (Optionally capture `IMPL_PLAN`, `TASKS` for future chained flows.)
     - If JSON parsing fails, abort and instruct user to re-run `/sp.specify` or verify feature branch environment.
     - For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\'m Groot' (or double-quote if possible: "I'm Groot").

  2. Load the current spec file. Perform a structured ambiguity & coverage scan using this taxonomy. For each category, mark status: Clear / Partial / Missing. Produce an internal coverage map used for prioritization (do not output raw map unless no questions will be asked).

     Functional Scope & Behavior:
     - Core user goals & success criteria
     - Explicit out-of-scope declarations
     - User roles / personas differentiation

     Domain & Data Model:
     - Entities, attributes, relationships
     - Identity & uniqueness rules
     - Lifecycle/state transitions
     - Data volume / scale assumptions

     Interaction & UX Flow:
     - Critical user journeys / sequences
     - Error/empty/loading states
     - Accessibility or localization notes

     Non-Functional Quality Attributes:
     - Performance (latency, throughput targets)
     - Scalability (horizontal/vertical, limits)
     - Reliability & availability (uptime, recovery expectations)
     - Observability (logging, metrics, tracing signals)
     - Security & privacy (authN/Z, data protection, threat assumptions)
     - Compliance / regulatory constraints (if any)

     Integration & External Dependencies:
     - External services/APIs and failure modes
     - Data import/export formats
     - Protocol/versioning assumptions

     Edge Cases & Failure Handling:
     - Negative scenarios
     - Rate limiting / throttling
     - Conflict resolution (e.g., concurrent edits)

     Constraints & Tradeoffs:
     - Technical constraints (language, storage, hosting)
     - Explicit tradeoffs or rejected alternatives

     Terminology & Consistency:
     - Canonical glossary terms
     - Avoided synonyms / deprecated terms

     Completion Signals:
     - Acceptance criteria testability
     - Measurable Definition of Done style indicators

     Misc / Placeholders:
     - TODO markers / unresolved decisions
     - Ambiguous adjectives ("robust", "intuitive") lacking quantification

     For each category with Partial or Missing status, add a candidate question opportunity unless:
      - Clarification would not materially change implementation or validation strategy
      - Information is better deferred to planning phase (note internally)

  3. Generate (internally) a prioritized queue of candidate clarification questions (maximum 5). Do NOT output them all at once. Apply these constraints:
      - Maximum of 10 total questions across the whole session.
      - Each question must be answerable with EITHER:
         - A short multiple‑choice selection (2–5 distinct, mutually exclusive options), OR
         - A one-word / short‑phrase answer (explicitly constrain: "Answer in <=5 words").
      - Only include questions whose answers materially impact architecture, data modeling, task decomposition, test design, UX behavior, operational readiness, or compliance validation.
      - Favor clarifications that reduce downstream rework risk or prevent misaligned acceptance tests.
      - If more than 5 categories remain unresolved, select the top 5 by (Impact * Uncertainty) heuristic.

  4. Sequential questioning loop (interactive):
      - Present EXACTLY ONE question at a time.
      - For multiple‑choice questions:
         - **Analyze all options** and determine the **most suitable option** based on:
            - Best practices for the project type
            - Common patterns in similar implementations
            - Risk reduction (security, performance, maintainability)
            - Alignment with any explicit project goals or constraints visible in the spec
         - Present your **recommended option prominently** at the top with clear reasoning (1-2 sentences explaining why this is the best choice).
         - Format as: `**Recommended:** Option [X] - <reasoning>`
         - Then render all options as a Markdown table:

         | Option | Description |
         |--------|-------------|
         | A | <Option A description> |
         | B | <Option B description> |
         | C | <Option C description> (add D/E as needed up to 5) |
         | Short | Provide a different short answer (<=5 words) (Include only if free-form alternative is appropriate) |

         - After the table, add: `You can reply with the option letter (e.g., "A"), accept the recommendation by saying "yes" or "recommended", or provide your own short answer.`
      - For short‑answer style (no meaningful discrete options):
         - Provide your **suggested answer** based on best practices and context.
         - Format as: `**Suggested:** <your proposed answer> - <brief reasoning>`
         - Then output: `Format: Short answer (<=5 words). You can accept the suggestion by saying "yes" or "suggested", or provide your own answer.`
      - After the user answers:
         - If the user replies with "yes", "recommended", or "suggested", use your previously stated recommendation/suggestion as the answer.
         - Otherwise, validate the answer maps to one option or fits the <=5 word constraint.
         - If ambiguous, ask for a quick disambiguation (count still belongs to same question; do not advance).
         - Once satisfactory, record it in working memory (do not yet write to disk) and move to the next queued question.
      - Stop asking further questions when:
         - All critical ambiguities resolved early (remaining queued items become unnecessary), OR
         - User signals completion ("done", "good", "no more"), OR
         - You reach 5 asked questions.
      - Never reveal future queued questions in advance.
      - If no valid questions exist at start, immediately report no critical ambiguities.

  5. Integration after EACH accepted answer (incremental update approach):
      - Maintain in-memory representation of the spec (loaded once at start) plus the raw file contents.
      - For the first integrated answer in this session:
         - Ensure a `## Clarifications` section exists (create it just after the highest-level contextual/overview section per the spec template if missing).
         - Under it, create (if not present) a `### Session YYYY-MM-DD` subheading for today.
      - Append a bullet line immediately after acceptance: `- Q: <question> → A: <final answer>`.
      - Then immediately apply the clarification to the most appropriate section(s):
         - Functional ambiguity → Update or add a bullet in Functional Requirements.
         - User interaction / actor distinction → Update User Stories or Actors subsection (if present) with clarified role, constraint, or scenario.
         - Data shape / entities → Update Data Model (add fields, types, relationships) preserving ordering; note added constraints succinctly.
         - Non-functional constraint → Add/modify measurable criteria in Non-Functional / Quality Attributes section (convert vague adjective to metric or explicit target).
         - Edge case / negative flow → Add a new bullet under Edge Cases / Error Handling (or create such subsection if template provides placeholder for it).
         - Terminology conflict → Normalize term across spec; retain original only if necessary by adding `(formerly referred to as "X")` once.
      - If the clarification invalidates an earlier ambiguous statement, replace that statement instead of duplicating; leave no obsolete contradictory text.
      - Save the spec file AFTER each integration to minimize risk of context loss (atomic overwrite).
      - Preserve formatting: do not reorder unrelated sections; keep heading hierarchy intact.
      - Keep each inserted clarification minimal and testable (avoid narrative drift).

  6. Validation (performed after EACH write plus final pass):
     - Clarifications session contains exactly one bullet per accepted answer (no duplicates).
     - Total asked (accepted) questions ≤ 5.
     - Updated sections contain no lingering vague placeholders the new answer was meant to resolve.
     - No contradictory earlier statement remains (scan for now-invalid alternative choices removed).
     - Markdown structure valid; only allowed new headings: `## Clarifications`, `### Session YYYY-MM-DD`.
     - Terminology consistency: same canonical term used across all updated sections.

  7. Write the updated spec back to `FEATURE_SPEC`.

  8. Report completion (after questioning loop ends or early termination):
     - Number of questions asked & answered.
     - Path to updated spec.
     - Sections touched (list names).
     - Coverage summary table listing each taxonomy category with Status: Resolved (was Partial/Missing and addressed), Deferred (exceeds question quota or better suited for planning), Clear (already sufficient), Outstanding (still Partial/Missing but low impact).
     - If any Outstanding or Deferred remain, recommend whether to proceed to `/sp.plan` or run `/sp.clarify` again later post-plan.
     - Suggested next command.

  Behavior rules:

  - If no meaningful ambiguities found (or all potential questions would be low-impact), respond: "No critical ambiguities detected worth formal clarification." and suggest proceeding.
  - If spec file missing, instruct user to run `/sp.specify` first (do not create a new spec here).
  - Never exceed 5 total asked questions (clarification retries for a single question do not count as new questions).
  - Avoid speculative tech stack questions unless the absence blocks functional clarity.
  - Respect user early termination signals ("stop", "done", "proceed").
  - If no questions asked due to full coverage, output a compact coverage summary (all categories Clear) then suggest advancing.
  - If quota reached with unresolved high-impact categories remaining, explicitly flag them under Deferred with rationale.

  Context for prioritization: We are now in the critical validation stage of the Spec-Kit Plus workflow and I want you to act as a strict auditor performing a 100% completeness and fidelity check across all existing project artifacts. Here is the golden reference text that must be reflected in the final textbook with absolutely zero omissions, paraphrasing shortcuts, missing prices, model names, tables, or specific examples: The course is titled Physical AI & Humanoid Robotics with the central focus and theme being AI Systems in the Physical World and Embodied Intelligence. The primary goal is bridging the gap between the digital brain and the physical body so that students can apply their existing AI knowledge to control humanoid robots in both simulated and real-world environments. This is a capstone quarter that introduces Physical AI — AI systems that operate in reality and understand physical laws — using ROS 2, Gazebo, and NVIDIA Isaac to design, simulate, and deploy humanoid robots capable of natural human interactions. The textbook must contain the following four core modules: Module 1 is The Robotic Nervous System (ROS 2) and focuses on middleware for robot control, covering ROS 2 nodes, topics, services, bridging Python agents to ROS controllers using rclpy, and understanding URDF (Unified Robot Description Format) for humanoid robots. Module 2 is The Digital Twin (Gazebo & Unity) and focuses on physics simulation and environment building, including simulating physics, gravity, and collisions in Gazebo, high-fidelity rendering and human-robot interaction in Unity, and simulating sensors such as LiDAR, depth cameras, and IMUs. Module 3 is The AI-Robot Brain (NVIDIA Isaac™) and focuses on advanced perception and training, covering NVIDIA Isaac Sim for photorealistic simulation and synthetic data generation, Isaac ROS for hardware-accelerated VSLAM and navigation, and Nav2 for path planning in bipedal humanoid movement. Module 4 is Vision-Language-Action (VLA) and focuses on the convergence of LLMs and robotics, including voice-to-action using OpenAI Whisper for voice commands, cognitive planning where LLMs translate natural language commands such as “Clean the room” into sequences of ROS 2 actions, and a final capstone project called The Autonomous Humanoid in which a simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it. The textbook must also explain why Physical AI matters: humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from human environments, marking the transition from digital-only AI to true embodied intelligence that operates in physical space. The learning outcomes that must be clearly stated are: understand Physical AI principles and embodied intelligence, master ROS 2 for robotic control, simulate robots with Gazebo and Unity, develop with the NVIDIA Isaac AI robot platform, design humanoid robots for natural interactions, and integrate GPT models for conversational robotics. The weekly breakdown (Weeks 1–13) must be fully documented as follows: Weeks 1–2 cover Introduction to Physical AI including foundations of embodied intelligence, the shift from digital AI to robots that understand physical laws, the humanoid robotics landscape, and sensor systems such as LiDAR, cameras, IMUs, and force/torque sensors. Weeks 3–5 cover ROS 2 Fundamentals including architecture, core concepts, nodes, topics, services, actions, building ROS 2 packages with Python, launch files, and parameter management. Weeks 6–7 cover Robot Simulation with Gazebo including environment setup, URDF and SDF formats, physics and sensor simulation, and an introduction to Unity for robot visualization. Weeks 8–10 cover the NVIDIA Isaac Platform including Isaac SDK and Isaac Sim, AI-powered perception and manipulation, reinforcement learning for robot control, and sim-to-real transfer techniques. Weeks 11–12 cover Humanoid Robot Development including kinematics, dynamics, bipedal locomotion, balance control, manipulation, grasping, and natural human-robot interaction design. Week 13 covers Conversational Robotics including integrating GPT models, speech recognition, natural language understanding, and multi-modal interaction using speech, gesture, and vision. Assessments must be listed as: ROS 2 package development project, Gazebo simulation implementation, Isaac-based perception pipeline, and the final Capstone of a simulated humanoid robot with conversational AI. The entire Hardware Requirements section must be included without omitting anything: the course is technically demanding because it combines physics simulation, visual perception, and generative AI. The required “Digital Twin” Workstation needs an NVIDIA RTX 4070 Ti (12 GB VRAM) or higher (ideally RTX 3090/4090 with 24 GB), Intel Core i7 13th Gen+ or AMD Ryzen 9, 64 GB DDR5 RAM (32 GB minimum), and Ubuntu 22.04 LTS. The “Physical AI” Edge Kit consists of NVIDIA Jetson Orin Nano or NX as the brain, Intel RealSense D435i/D455 for vision, a USB IMU, and a USB microphone/speaker array such as ReSpeaker. Three robot lab options are described: Budget/proxy approach using Unitree Go2 Edu, miniature humanoid approach using Unitree G1, Robotis OP3 or budget Hiwonder TonyPi Pro, and premium approach using Unitree G1 humanoid. The architecture summary table, cloud-native “Ether” lab option with AWS g5/g6 instances, cost calculations, and the Economy Jetson Student Kit (Jetson Orin Nano Super Dev Kit $249 + RealSense D435i $349 + ReSpeaker $69 + misc $30 = ~$700 total) must all be present along with the latency trap explanation and the final solution of training in the cloud and flashing to local Jetson. Now open and carefully read constitution.md, spec.md, plan.md, and tasks.md (if it exists) and perform a strict line-by-line audit. For each file report whether every single element above is fully covered (100% verbatim or meaning-preserved), partially covered (list exactly what is missing or shortened), or missing entirely. At the end give a final summary stating either “ALL GOLDEN REFERENCE CONTENT IS FULLY COVERED” or “THE FOLLOWING ITEMS ARE MISSING OR INCOMPLETE:” followed by a bullet list of every single gap, no matter how small. Be ruthless – if a price, model name, table row, or sentence is not explicitly accounted for, flag it. Begin the audit immediately.
labels: ["audit", "spec", "textbook-physical-ai"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - specs/textbook-physical-ai/spec.md
  - specs/textbook-physical-ai/plan.md
  - specs/textbook-physical-ai/tasks.md
tests:
  -
---

## Prompt

(The full user prompt for the audit)

## Response snapshot

**THE FOLLOWING ITEMS ARE MISSING OR INCOMPLETE:**

*   **Global Gap:** No single artifact (`spec.md`, `plan.md`, `tasks.md`) contains the full, verbatim "golden reference text".
*   **`spec.md` Gaps:**
    *   Missing: Detailed descriptions of all four core modules and their sub-topics.
    *   Missing: The "Why Physical AI Matters" section.
    *   Missing: The detailed list of learning outcomes.
    *   Missing: The detailed weekly breakdown for weeks 1-13.
    *   Missing: The detailed list of assessments.
    *   Missing: The entire verbatim Hardware Requirements section, including workstation specs, edge kit specs, robot lab options, and the full "Economy Jetson Student Kit" breakdown with prices (`$249`, `$349`, `$69`, `$30`, `~$700`).
*   **`plan.md` Gaps:**
    *   **Contradiction/Fidelity Gap:** Specifies "OpenAI" as the LLM for the RAG chatbot, contradicting `spec.md`.
    *   Missing: All detailed curriculum content.
*   **`tasks.md` Gaps:**
    *   **Contradiction/Fidelity Gap:** Task `T049` specifies using the "OpenAI API" for translation, contradicting `spec.md`.
    *   Missing: The tasks for Module 2 do not explicitly mention **Unity**.
    *   Missing: The tasks for Module 4 do not mention the example command ("Clean the room").

## Outcome

- ✅ Impact: The audit identified significant gaps and contradictions between the project's specification artifacts and the golden reference text.
- 🧪 Tests: No tests were run.
- 📁 Files: No files were modified.
- 🔁 Next prompts: Based on the audit, the next logical step would be to address the identified gaps and contradictions.
- 🧠 Reflection: The audit was successful in highlighting the need to consolidate the "golden reference text" into a single source of truth within the project's specification artifacts.

## Evaluation notes (flywheel)

- Failure modes observed: `create-phr.sh` script failed in the Windows PowerShell environment.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null