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

## 🌐 Deployment

### Frontend (Vercel/GitHub Pages)
The Docusaurus frontend can be deployed to:
- Vercel: Connect your GitHub repository
- GitHub Pages: Use the built-in deployment workflow

### Backend (Render/Docker)
The FastAPI backend is designed for deployment on:
- Render: Use the provided `render.yaml`
- Other platforms: Containerized with the provided `Dockerfile`

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