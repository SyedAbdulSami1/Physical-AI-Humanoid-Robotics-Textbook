---
id: 0003
title: Clarify Textbook Spec
stage: spec
date: 2025-12-08
surface: agent
model: gemini
feature: textbook-physical-ai
branch: main
user: unknown
command: /sp.clarify
labels: [clarification, spec]
links:
  spec: specs/textbook-physical-ai/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/textbook-physical-ai/spec.md
  - history/prompts/textbook-physical-ai/0003-clarify-textbook-spec.spec.prompt.md
tests:
  (none)
---

## Prompt

Identify underspecified areas in the current feature spec by asking up to 5 highly targeted clarification questions and encoding answers back into the spec.

Context for prioritization: Act as the Clarify Phase detective and thoroughly analyze the current book specification for the Hackathon I Physical AI & Humanoid Robotics Textbook project (Docusaurus + RAG Chatbot + Bonus Features) located at specs/book/spec.md, examining four categories with extreme detail and ruthlessness since this book will be judged in a high-stakes hackathon and used to teach a university-level course where nothing can remain vague. Category 1 AMBIGUOUS TERMS: identify terms that can be interpreted in multiple ways including what exactly counts as "AI-native textbook", what does "properly deployed on GitHub Pages" mean (custom domain? HTTPS? CI? mobile-friendly? dark mode?), what level of "performance" is expected from the RAG chatbot (latency? accuracy? hallucination rate?), what does "personalise the content" mean for bonus #6 (difficulty level? examples? add/remove sections? tone?), and any other vague adjectives like "excellent", "smooth", "production-ready", "fully functional", etc. Category 2 MISSING ASSUMPTIONS: identify things probably assumed but never written down including target audience skill level (complete beginners? already know Python/ROS? have RTX GPUs?), primary language of the book (English only, or Urdu translation bonus is mandatory to attempt?), exact version requirements (Node.js, Docusaurus, Python, ROS 2 Humble/Iron/Jazzy, Isaac Sim 2024.x?), hosting expectations (only GitHub Pages, or Vercel/Netlify also acceptable?), mobile responsiveness required?, do judges expect the site to be indexed by Google / SEO-friendly?, authentication (Better-Auth) is email/password enough or OAuth (Google/GitHub) expected?, database (Neon Postgres) is the free tier enough or do we need connection pooling for many users? Category 3 INCOMPLETE REQUIREMENTS & MISSING EDGE CASES: for RAG chatbot does it have to handle selected-text-only questions perfectly?, must it refuse to answer if no context is provided?, streaming responses required?, conversation history persistence?; for personalisation button what exact questions are asked at signup to determine background?, how is content actually personalised (show/hide sections? change explanation depth? add beginner notes?); for Urdu translation button real-time translation or pre-translated Markdown files?, which translation model (GPT-4o, Gemini, NLLB, custom?)?, RTL layout support needed?; for bonus features scoring are the 50-point bonuses completely independent or do they have dependencies?; for capstone & hardware sections do we need working code snippets for Jetson + RealSense setup?, are cost tables expected to be up-to-date as of Nov 2025? Category 4 SCOPE CONFLICTS & CONTRADICTORY STATEMENTS: the project says "unified book project" but also has four 50-point bonuses so is attempting all bonuses required for top score or are top submissions possible with only core + 1-2 bonuses?, "Deploy to GitHub Pages" vs some Docusaurus plugins (search, auth) that don't work properly on GitHub Pages so any known conflicts?, deadline is 30 Nov 2025 6 PM so does the site need to be live and stable by that exact second? For each finding clearly mark it as CRITICAL (will break planning, implementation or judging if not clarified now), IMPORTANT (strongly affects quality or bonus points), or NICE-TO-HAVE (can be decided later). Finally give a short verdict stating "Is this specification currently clear enough to safely proceed to /sp.plan, or do I need at least one more clarification round?" and be as strict as possible since this is a hackathon with startup founder opportunity on the line.

---

## Response snapshot

I have successfully analyzed the `spec.md` file and asked 5 targeted clarification questions to the user. All answers have been integrated back into the `specs/textbook-physical-ai/spec.md` file.

**Final Report:**

*   **Number of questions asked & answered:** 5
*   **Path to updated spec:** `specs/textbook-physical-ai/spec.md`
*   **Sections touched:** `## Clarifications`, `## Success Evals`, `## User Personas`, `## Constraints`

**Coverage Summary Table:**

| Category                             | Status      |
| :----------------------------------- | :---------- |
| Functional Scope & Behavior          | Resolved    |
| Domain & Data Model                  | Clear       |
| Interaction & UX Flow                | Clear       |
| Non-Functional Quality Attributes    | Partial     |
| Integration & External Dependencies  | Resolved    |
| Edge Cases & Failure Handling        | Resolved    |
| Constraints & Tradeoffs              | Resolved    |
| Terminology & Consistency            | Clear       |
| Completion Signals                   | Clear       |
| Misc / Placeholders                  | Clear       |

**Verdict:** The specification is now significantly clearer. The critical ambiguities regarding personalization, RAG chatbot behavior, target audience, core software versions, and Urdu translation methods have been resolved. This level of clarity is sufficient to safely proceed to the `/sp.plan` phase.

**Suggested next command:** `/sp.plan`

## Outcome

- ✅ Impact: Clarified critical ambiguities in the feature specification, reducing rework risk for planning and implementation.
- 🧪 Tests: (none)
- 📁 Files: Updated `specs/textbook-physical-ai/spec.md` and created this PHR.
- 🔁 Next prompts: The user can now proceed with `/sp.plan`.
- 🧠 Reflection: The process of asking targeted questions and immediately integrating answers into the spec proved effective for incremental clarification and reducing ambiguity.

## Evaluation notes (flywheel)

- Failure modes observed: (none)
- Graders run and results (PASS/FAIL): (none)
- Prompt variant (if applicable): (none)
- Next experiment (smallest change to try): (none)