# ADR-0008: Deployment Platform

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** The project requires a deployment platform for both the static Docusaurus frontend and the FastAPI backend. The ideal solution should be easy to set up, offer a generous free tier, provide CI/CD integration, and handle the distinct requirements of a static site and a dynamic backend service.

## Decision

We will adopt a **hybrid deployment strategy**.
1.  **Frontend (Docusaurus):** The static site will be deployed to **GitHub Pages**. This is a free and robust solution for hosting static files directly from a GitHub repository. A GitHub Actions workflow will be created to build the Docusaurus site and deploy the `build` output to the `gh-pages` branch.
2.  **Backend (FastAPI):** The Python backend will be deployed as a containerized service on a platform like **Render** or a similar "Platform as a Service" (PaaS). Render offers a free tier for web services, can build and deploy from a `Dockerfile`, and provides a public URL for the API.

## Consequences

### Positive

- **Cost-Effective:** Both GitHub Pages and the free tiers of PaaS providers like Render are free, eliminating hosting costs for the hackathon.
- **Optimized for Purpose:** Each component is hosted on a platform designed for its specific needs (static hosting for the frontend, service hosting for the backend).
- **CI/CD Integration:** GitHub Actions provides a powerful and integrated way to automate the build and deployment process for both the frontend and backend.
- **Scalability:** Both platforms offer clear paths to scale up if the project's needs grow beyond the free tiers.

### Negative

- **Slightly More Complex Setup:** Managing two separate deployment pipelines (one for frontend, one for backend) is slightly more complex than a monorepo deployment on a unified platform like Vercel.
- **CORS Configuration:** We will need to correctly configure Cross-Origin Resource Sharing (CORS) on the FastAPI backend to allow requests from the frontend hosted on a different domain (the `github.io` domain).

## Alternatives Considered

- **Vercel:** Vercel offers an all-in-one solution that can deploy a Docusaurus frontend and also run a Python backend using Vercel Functions.
    - **Why Rejected:** While Vercel is excellent, its primary strength is its seamless integration with Next.js. While it supports Python serverless functions, the free tier has limitations (e.g., execution time) that might be restrictive for a potentially long-running translation or RAG process. The hybrid approach gives us more flexibility and control over the backend environment.
- **Netlify:** Similar to Vercel, Netlify is a strong platform for static sites with serverless function capabilities.
    - **Why Rejected:** The reasoning is similar to Vercel. While a valid option, separating the concerns allows us to choose the best-in-class free service for each part of our stack (GitHub Pages for static content, a dedicated backend PaaS for the API).
- **Self-hosting on a VPS (e.g., DigitalOcean, Linode):** Deploying both the frontend and backend to a single virtual private server.
    - **Why Rejected:** This requires manual setup of a web server (like Nginx), process management for the FastAPI app (like Gunicorn/Uvicorn), and manual setup of SSL certificates. This is far too much operational overhead for a hackathon.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
