# Common Pitfalls and Fixes for Physical AI & Humanoid Robotics Textbook RAG Chatbot

## Backend Issues

### 1. Qdrant Connection Issues
**Problem:** Qdrant client fails to connect or authentication errors
**Solution:** 
- Verify QDRANT_URL and QDRANT_API_KEY in your .env file
- For Qdrant Cloud: Use the full URL including port (e.g., `https://your-cluster-id.europe-west3-0.gcp.cloud.qdrant.io:6333`)
- For local Qdrant: Ensure the service is running (`docker run -p 6333:6333 -p 6334:6334 qdrant/qdrant`)
- Check firewall settings if connecting to cloud Qdrant

### 2. OpenAI API Connection Issues
**Problem:** OpenAI API calls fail with authentication errors
**Solution:**
- Verify OPENAI_API_KEY in your .env file is valid
- Check that your OpenAI account has sufficient credits
- Ensure the API key has the necessary permissions for embeddings and chat completion

### 3. Database Connection Issues
**Problem:** PostgreSQL/Neon connection fails
**Solution:**
- Verify NEON_DATABASE_URL format: `postgresql://username:password@ep-xxxxxx.region.provider.neon.tech/dbname?sslmode=require`
- Update the connection string with your actual Neon credentials
- Check that your Neon instance is running and accessible

### 4. Vector Embedding Mismatch
**Problem:** RAG system fails to find relevant content during search
**Solution:**
- Ensure the same embedding model is used for both ingestion and querying (text-embedding-ada-002)
- Verify that content is properly chunked during ingestion to maintain context
- Check that Qdrant collection vectors_config matches the embedding size (1536 for Ada-002)

## Frontend Issues

### 5. CORS Errors
**Problem:** Cross-origin requests fail between frontend and backend
**Solution:**
- Update CORS middleware in main.py to include your frontend domain:
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "https://yourdomain.com"],  # Add your domains
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### 6. Authentication Token Issues
**Problem:** Auth tokens not being stored or sent correctly
**Solution:**
- Ensure JWT_SECRET_KEY in .env is secure and consistent
- Verify that authentication endpoints are being called correctly
- Check that tokens are properly stored in localStorage or cookies

### 7. Component Bundle Errors
**Problem:** Docusaurus components don't load or show build errors
**Solution:**
- Run `npm install` to ensure all dependencies are available
- Verify component file paths match the import statements
- Check that CSS files are properly imported

## RAG Performance Issues

### 8. Slow Response Times
**Problem:** Chatbot responses take too long
**Solutions:**
- Implement caching for repeated queries
- Use semantic search with appropriate top_k values (5-10 is typically optimal)
- Optimize content chunking size (500-1000 tokens is usually effective)
- Consider using faster embedding models for initial retrieval

### 9. Poor Relevance in Results
**Problem:** Retrieved content is not relevant to the query
**Solutions:**
- Improve query refinement before vector search
- Implement re-ranking after initial retrieval
- Use hybrid search combining semantic and keyword search
- Fine-tune the prompt engineering for better context utilization

## Deployment Issues

### 10. Environment Variables Missing in Production
**Problem:** Application fails in production due to missing environment variables
**Solutions:**
- For Vercel: Set environment variables in the Vercel dashboard
- For GitHub Pages: Use build-time environment variables
- For self-hosted: Use proper secrets management

### 11. Static Assets Not Loading
**Problem:** CSS, images, or other assets fail to load after deployment
**Solutions:**
- Verify the `baseUrl` setting in docusaurus.config.ts
- Check that asset paths are relative to the correct directory
- Ensure build process includes all necessary assets

## Translation Issues

### 12. Urdu Translation Failures
**Problem:** Translation API calls fail or return incorrect results
**Solutions:**
- Verify Google Cloud Translation API or DeepL API credentials
- Check if the target language code is correct ('ur' for Urdu)
- Implement fallback translation mechanisms
- Add proper error handling for translation timeouts

## Personalization Issues

### 13. Personalization Not Persisting
**Problem:** User preferences are lost between sessions
**Solutions:**
- Verify that user preference data is being stored correctly in the database
- Check that JWT tokens include proper user identification
- Ensure frontend is making correct API calls to retrieve user preferences

## Testing Issues

### 14. RAG Accuracy Below Threshold
**Problem:** RAG accuracy test fails to meet >90% requirement
**Solutions:**
- Improve content chunking to maintain context better
- Enhance query refinement and routing logic
- Implement better response verification mechanisms
- Add more diverse test cases to cover different scenarios

## Scaling Issues

### 15. High Memory Usage
**Problem:** Application consumes too much memory during processing
**Solutions:**
- Implement streaming responses for large content
- Use pagination for large content retrieval
- Optimize database queries with proper indexing
- Implement content caching strategies

### 16. Rate Limiting
**Problem:** API calls hitting rate limits (especially OpenAI)
**Solutions:**
- Implement request queuing and rate limiting
- Add retry logic with exponential backoff
- Cache expensive operations where appropriate
- Consider using multiple API keys for load distribution

## Development Workflow Tips

### 17. Local Development Setup
- Use virtual environments to isolate dependencies
- Store sensitive credentials in .env files (and add to .gitignore)
- Run backend and frontend separately during development
- Use proper logging to debug issues

### 18. Debugging Strategies
- Add comprehensive logging throughout the RAG pipeline
- Use middleware to log request/response cycles
- Implement health check endpoints for monitoring
- Add performance metrics to track response times

## Security Considerations

### 19. Input Validation
- Always validate user inputs before processing
- Sanitize content before storage and display
- Implement proper authentication and authorization
- Use parameterized queries to prevent SQL injection

### 20. Authentication Flow
- Secure JWT tokens with proper expiration
- Implement secure password hashing (bcrypt)
- Use HTTPS in production
- Implement proper session management