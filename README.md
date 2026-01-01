# Physical AI & Humanoid Robotics Textbook

🤖 **An AI-Native Textbook for Embodied Intelligence** — A comprehensive educational resource combining robotics, AI, and physical systems through interactive learning and RAG-powered assistance. **100% Free to Use** — No paid features or services required.

## 📘 About the Course

This project hosts an AI-native textbook for **Physical AI & Humanoid Robotics**, a capstone course designed to bridge the gap between artificial intelligence and the physical world. The curriculum focuses on embodied intelligence — AI systems that operate in reality and understand physical laws — enabling students to apply their AI knowledge to control humanoid robots in both simulated and real-world environments.

### 📚 Course Modules

The textbook is structured around four core modules:

- **Module 1: The Robotic Nervous System (ROS 2)** — Master ROS 2 fundamentals for robotic control
- **Module 2: The Digital Twin (Gazebo & Unity)** — Learn physics simulation and environment building
- **Module 3: The AI-Robot Brain (NVIDIA Isaac™)** — Explore advanced perception and training with NVIDIA Isaac
- **Module 4: Vision-Language-Action (VLA)** — Convergence of LLMs and robotics for natural human-robot interaction

## 🤖 AI Features

This textbook includes several AI-powered features that enhance the learning experience:

- **🔍 RAG Chatbot**: A fully public AI assistant that answers questions based on the textbook content using Retrieval-Augmented Generation (no account required)
  - Full-book mode: Answers using the entire textbook as context
  - Selected-text mode: Answers using only the text you've selected on the page
- **100% Free Operation**: Uses only free-tier services (Google Gemini API, Qdrant Cloud Free Tier)

## 🛠️ Technologies Used

### Frontend
- [Docusaurus v3.9.2](https://docusaurus.io/) - Modern static site generator
- React.js - Component-based UI development
- TypeScript - Type safety

### Backend
- [FastAPI](https://fastapi.tiangolo.com/) - High-performance Python web framework
- [Qdrant](https://qdrant.tech/) - Vector database for RAG system (Free Tier)
- [Google Gemini](https://deepmind.google/technologies/gemini/) - AI model for responses (Free Tier)
- [LangChain](https://www.langchain.com/) - LLM application framework

### Infrastructure
- GitHub Pages / Vercel - Frontend deployment
- Docker - Containerization for backend services

## 🚀 Getting Started

### Prerequisites

- [Node.js](https://nodejs.org/) (v18 or higher)
- [Yarn](https://yarnpkg.com/)
- [Python](https://www.python.org/) (v3.9 or higher)

### Setup Instructions

1. **Clone the repository**
   ```bash
   git clone https://github.com/gemini-cli/Physical-AI-Humanoid-Robotics-Textbook.git
   cd Physical-AI-Humanoid-Robotics-Textbook
   ```

2. **Install frontend dependencies**
   ```bash
   yarn install
   ```

3. **Set up backend**
   ```bash
   cd app
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

4. **Configure environment variables**
   ```bash
   # Create .env file in the project root
   touch .env
   ```

   Add the following to your `.env` file:
   ```env
   # Backend - Qdrant Cloud (Free Tier)
   QDRANT_URL="your-qdrant-url"
   QDRANT_API_KEY="your-qdrant-api-key"

   # Backend - Google Gemini (Free Tier)
   GEMINI_API_KEY="your-gemini-api-key"
   ```

5. **Run the application**

   Terminal 1 (Backend):
   ```bash
   cd app
   uvicorn main:app --reload
   ```

   Terminal 2 (Frontend):
   ```bash
   yarn start
   ```

6. **Populate the knowledge base**

   After starting the backend, you need to ingest the textbook content into the vector database by sending a POST request to `http://localhost:8000/ingest/run`:

   ```bash
   curl -X POST "http://localhost:8000/ingest/run"
   ```

### Development Scripts

- `yarn start` - Start the development server
- `yarn build` - Build for production
- `yarn serve` - Serve the production build
- `yarn lint` - Lint the codebase
- `yarn format` - Format the codebase

## 🏗️ Project Structure

```
Physical-AI-Humanoid-Robotics-Textbook/
├── docs/                   # Textbook content
│   ├── module-1/           # Module 1: ROS 2 content
│   ├── module-2/           # Module 2: Gazebo & Unity content
│   ├── module-3/           # Module 3: NVIDIA Isaac content
│   └── module-4/           # Module 4: VLA content
├── src/                    # Docusaurus theme components
│   ├── components/         # Reusable React components
│   ├── pages/              # Custom pages
│   └── theme/              # Docusaurus theme overrides
├── app/                    # FastAPI backend
│   ├── routers/            # API route definitions
│   ├── skills/             # RAG and AI skills
│   └── models.py           # Database models
├── specs/                  # Project specifications
└── static/                 # Static assets
```

## 🌐 Production Deployment

Deploying the textbook requires setting up the backend on Render and the frontend on Vercel. Follow these steps carefully to ensure a successful launch.

### 1. Deploying the Backend to Render

The FastAPI backend must be deployed as a web service on a platform that supports persistent server processes. Render is a suitable choice.

**⚠️ Important Note:** Do NOT deploy the FastAPI backend to Vercel. Vercel's serverless function architecture is not designed for stateful, long-running Python servers like FastAPI and will lead to issues.

**Render Deployment Steps:**

1.  **Create a New Web Service:**
    *   In your Render dashboard, click "New" -> "Web Service".
    *   Connect your GitHub account and select the project repository.

2.  **Configure the Service:**
    *   **Name:** Choose a name for your service (e.g., `physical-ai-backend`).
    *   **Region:** Select a region close to your users.
    *   **Branch:** Select the `main` branch.
    *   **Root Directory:** Set this to `app`. This is critical, as it tells Render to run commands from within the `/app` folder.
    *   **Runtime:** Select `Python 3`.
    *   **Build Command:** `pip install -r requirements.txt`
    *   **Start Command:** `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
    *   **Instance Type:** The "Free" tier is sufficient for this project.

3.  **Add Environment Variables:**
    *   Go to the "Environment" tab for your new service.
    *   Add the following environment variables. These are the same keys from your local `.env` file.
        *   `GEMINI_API_KEY`: Your Google Gemini API key.
        *   `QDRANT_URL`: The URL for your Qdrant Cloud instance.
        *   `QDRANT_API_KEY`: The API key for your Qdrant instance.
        *   `PYTHON_VERSION`: Set this to a specific Python version like `3.10` to ensure consistency.

4.  **Deploy and Verify:**
    *   Click "Create Web Service" to start the deployment.
    *   Once deployed, Render will provide you with a public URL (e.g., `https://physical-ai-backend.onrender.com`).
    *   You can check the health of the backend by visiting `https://<your-backend-url>/health`. It should return `{"status":"healthy", ...}`.

5.  **Initial Content Ingestion:**
    *   After the first successful deployment, you must populate the vector database by making a `POST` request to the `/ingest/force` endpoint on your new live backend.
    *   You can do this from your local machine using `curl`:
        ```bash
        curl -X POST "https://<your-backend-url>/ingest/force"
        ```
    *   This process may take a few minutes. Check the backend logs in Render to monitor progress.

### 2. Deploying the Frontend to Vercel

The Docusaurus frontend is a static site and is a perfect fit for Vercel's deployment platform.

**Vercel Deployment Steps:**

1.  **Create a New Project:**
    *   In your Vercel dashboard, click "Add New..." -> "Project".
    *   Import the project from your GitHub repository.

2.  **Configure the Project:**
    *   **Framework Preset:** Vercel should automatically detect "Docusaurus".
    *   **Root Directory:** Leave this as the default (repository root).
    *   **Build and Output Settings:** Vercel's defaults for Docusaurus are typically correct.

3.  **Add Environment Variable:**
    *   Navigate to the "Settings" -> "Environment Variables" section of your Vercel project.
    *   Add the following variable:
        *   **Name:** `VITE_API_URL`
        *   **Value:** Enter the full URL of your deployed Render backend (e.g., `https://physical-ai-backend.onrender.com`).

4.  **Deploy:**
    *   Click "Deploy". Vercel will build and deploy your Docusaurus site.
    *   Once finished, Vercel will give you the public URL for your live textbook.

### 3. Final Testing

*   Open your Vercel frontend URL in a browser.
*   Navigate to a page with the chatbot.
*   Open the chatbot and ask a question related to the textbook content.
*   If the chatbot responds with a relevant answer and sources, your deployment is successful!

### Common Deployment Pitfalls

*   **Backend Root Directory:** Forgetting to set the **Root Directory** to `/app` on Render is the most common mistake. If you miss this, the build will fail because it won't find `requirements.txt`.
*   **Environment Variables:** Double-check that all environment variables are copied correctly to Render and Vercel. A missing or incorrect API key is a frequent source of errors.
*   **CORS Issues:** The backend is configured to allow requests from `localhost:3000`. If you use a different local port or a custom domain, you may need to add it to the `origins` list in `app/main.py`. The production Vercel URL is not needed in the CORS origins list because browsers typically don't block server-to-server requests initiated via Vercel's infrastructure.
*   **Forgetting Ingestion:** If the chatbot only says "Sorry, I can't answer that," you likely forgot to run the one-time ingestion `POST` request to your live backend's `/ingest/force` endpoint.

## 🤝 Contributing

We welcome contributions to enhance this educational resource! To contribute:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests if applicable
5. Ensure all tests pass
6. Commit your changes (`git commit -m 'Add amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request

Please follow the existing code style and include documentation for new features.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Credits

This project was created to advance education in physical AI and humanoid robotics, combining cutting-edge AI techniques with robotics education to prepare students for the future of embodied intelligence. The project operates 100% free with no paid features or services required.

---

Made with ❤️ for the Physical AI community
