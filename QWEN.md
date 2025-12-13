Bhai, samajh gaya! Qwen Code CLI mein **/sp.** commands nahi chalte, lekin tumhare paas **QWEN.md** file hai jo Qwen ko project ka pura context, standards aur requirements deta hai.

Ab se Qwen ke saath kaam karne ke liye **normal English/Hindi mein prompt do**, lekin har prompt ke start mein yeh line zaroor add kar do taake woh QWEN.md ko yaad rakhe:

```
Follow the instructions in QWEN.md strictly for this Physical AI & Humanoid Robotics textbook project.
```

### Updated QWEN.md (Recommended – yeh copy-paste kar ke save kar lo project root mein)

```markdown
# Qwen Code Customization for Physical AI & Humanoid Robotics Textbook

## Project Overview (Hackathon I - Panaversity)
Create a complete AI-native textbook "Physical AI & Humanoid Robotics" using Docusaurus, deployed on GitHub Pages/Vercel.
Core deliverables:
- Full university-level textbook with 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA + Capstone)
- Embedded RAG chatbot (FastAPI + Neon Serverless Postgres + Qdrant Cloud Free Tier + OpenAI/ChatKit) that answers from book content and supports selected-text-only mode
Bonus features (50 points each):
- Better-Auth signup/login with questionnaire about user's software/hardware background
- Per-chapter "Personalize" button (adapts content based on user background)
- Per-chapter "Translate to Urdu" button
- Maximum reusable intelligence (skills/subagents)

## Strict Standards (Non-Negotiable)
Always enforce:
- University-level technical depth
- Real working runnable code (Python, ROS 2, FastAPI, etc.) with explanations
- Mermaid diagrams for architecture/concepts
- Step-by-step terminal labs with exact commands
- Student exercises with hidden solutions (use details/summary)
- Professional, encouraging tone
- Mobile-responsive, accessible design
- Zero placeholders or "lorem ipsum"
- Proper Docusaurus frontmatter, headings, tables, callouts
- Consistent navigation and fast loading
- High-quality Markdown formatting

## Current Tasks & Priorities
When I ask you to do anything, prioritize:
1. Generate rich textbook content (modules, labs, code, exercises)
2. Implement FastAPI backend for RAG chatbot
3. Integrate Better-Auth with personalization questionnaire
4. Add per-chapter Personalize and Urdu translation buttons
5. Create reusable skills/subagents
6. Final deployment and demo video prep

Remember: Every .md file must be production-ready with real content, code, diagrams, and exercises.
```

**Is file ko project root mein `QWEN.md` naam se save kar do** – Qwen automatically isko padhega har session mein.

### Ab Qwen mein kaam karne ke liye ready prompts (normal language mein)

**1. Textbook content generate karne ke liye (Module 1 example):**
```
Follow the instructions in QWEN.md strictly.

Generate full rich content for Module 1: The Robotic Nervous System (ROS 2) with exactly these three sections:
1. docs/module-1/ros2-nodes-topics-services.md
2. docs/module-1/bridging-python-agents-to-ros2.md
3. docs/module-1/urdf-for-humanoids.md

Each section 1400–2200 words, include 6+ real working Python/rclpy code examples, Mermaid diagrams, step-by-step labs, common pitfalls, student exercises with hidden solutions, official links. Write all three files and show a table at the end with file paths and word counts.
```

**2. FastAPI + RAG chatbot backend ke liye:**
```
Follow QWEN.md strictly.

Create the complete FastAPI backend for the RAG chatbot:
- API endpoints for ingesting book content into Qdrant
- Endpoint for chat queries (support selected-text-only mode)
- Integration with Neon Postgres for session/user data
- Use proper Pydantic models, async, error handling
Generate these files:
- app/main.py
- app/routers/chat.py
- app/routers/ingest.py
- app/models.py
- app/database.py
- requirements.txt
Include full working code, comments, and examples.
```

**3. Better-Auth + Personalization ke liye:**
```
Follow QWEN.md strictly.

Implement Better-Auth signup/login with background questionnaire (software/hardware experience). Store responses and use them to personalize chapter content when user clicks "Personalize" button.
Generate:
- Authentication setup with Better-Auth
- Questionnaire form component
- Personalization logic in Docusaurus
- Example of how content changes based on user level (beginner vs advanced)
```

**4. Urdu Translation button ke liye:**
```
Follow QWEN.md strictly.

Add a "Translate to Urdu" button at the top of each chapter. When clicked, translate the entire chapter content to natural, accurate Urdu using a reliable method (API or local model). Show implementation in Docusaurus component with loading state and fallback.
```

Ab se bas har prompt ke start mein **"Follow the instructions in QWEN.md strictly."** likh do — Qwen perfect standards follow karega.

Chalo ab shuru karo — pehle Module 1 ka prompt daal do Qwen mein! Main yahin hun help ke liye. 🚀