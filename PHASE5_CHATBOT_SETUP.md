# Phase 5: RAG Chatbot Setup Guide

This guide explains how to use the RAG-powered chatbot widget in the AI-Native Textbook.

## Features

- **Interactive Chat Widget**: Fixed position in bottom-right corner
- **RAG-Powered Responses**: Answers based on textbook content using vector search
- **Source Citations**: Shows which chapters/sections were used to generate answers
- **Dark Mode Support**: Automatically adapts to Docusaurus theme
- **Mobile Responsive**: Works on all devices

## Architecture

```
┌─────────────┐          ┌──────────────┐          ┌─────────────┐
│   React     │          │   FastAPI    │          │   Qdrant    │
│  ChatWidget │  ────>   │   Backend    │  ────>   │   Vector    │
│  Component  │          │   (RAG)      │          │   Database  │
└─────────────┘          └──────────────┘          └─────────────┘
```

## Prerequisites

Before using the chatbot, you need:

1. **Backend Running** with Qdrant vector database initialized
2. **Textbook Content Ingested** into Qdrant
3. **Frontend Development Server** or static build

## Step 1: Start the Backend

```bash
cd backend

# Install dependencies
pip install -r requirements.txt

# Create .env file (copy from .env.example)
cp .env.example .env

# Edit .env and add your API keys:
# - NEON_DB_URL (Postgres)
# - QDRANT_URL and QDRANT_API_KEY
# - OPENAI_API_KEY (OpenRouter)

# Run the backend
python -m src.main
```

The backend should start on `http://localhost:8000`.

Verify it's running:
```bash
curl http://localhost:8000/health
```

## Step 2: Ingest Textbook Content

The chatbot needs textbook content in the vector database. Run the ingestion script:

```bash
cd backend

# Ingest all textbook chapters
python scripts/ingest_textbook.py

# Or ingest specific chapters
python scripts/ingest_textbook.py --chapter intro
```

This will:
- Read all `.md` files from `frontend/docs/`
- Split content into chunks
- Generate embeddings using OpenAI
- Store in Qdrant vector database

**Note**: Ingestion costs approximately $0.10-0.50 depending on content size.

## Step 3: Configure CORS (if needed)

If your frontend runs on a different port or domain, update the CORS configuration:

**backend/.env**
```bash
# For local development
CORS_ORIGINS=http://localhost:3000,http://localhost:3001

# For production (GitHub Pages)
CORS_ORIGINS=https://aiman-17.github.io
```

## Step 4: Start the Frontend

```bash
cd frontend

# Install dependencies
npm install

# Start development server
npm start
```

The site should open at `http://localhost:3000`.

## Using the Chatbot

1. Look for the **purple chat bubble** in the bottom-right corner
2. Click it to open the chat window
3. Type your question about the textbook
4. Press Enter or click the send button
5. View the AI-generated answer with source citations

### Example Questions

- "What is Physical AI?"
- "How does ROS 2 work?"
- "Explain SLAM in robotics"
- "What are the components of a humanoid robot?"
- "How do I set up NVIDIA Isaac Sim?"

## Configuration

### Changing the Backend URL

Edit `frontend/src/components/ChatWidget/index.tsx`:

```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-production-backend.com'  // Change this
  : 'http://localhost:8000';
```

### Adjusting Chat Behavior

In `backend/src/api/routers/chat.py`, you can modify:

- **Number of sources**: Change `max_results` in the query (default: 3)
- **Similarity threshold**: Change `score_threshold` (default: 0.7)
- **Response length**: Change `max_tokens` in OpenAI call (default: 800)
- **Temperature**: Change creativity level (default: 0.7)

## Troubleshooting

### Chatbot Not Appearing

1. Check browser console for errors
2. Verify `Root.tsx` is properly importing ChatWidget
3. Clear browser cache and rebuild: `npm run clear && npm run build`

### "Failed to get response" Error

1. **Backend not running**: Verify `http://localhost:8000/health` responds
2. **CORS error**: Check browser console, update CORS_ORIGINS in .env
3. **Qdrant not connected**: Check backend logs for Qdrant connection errors

### "No relevant information found"

1. **Content not ingested**: Run `python scripts/ingest_textbook.py`
2. **Query too specific**: Try broader questions
3. **Similarity threshold too high**: Lower `score_threshold` in chat.py

### High API Costs

1. **Enable caching**: Translation cache is already implemented for Phase 4
2. **Limit max_tokens**: Reduce response length
3. **Use cheaper model**: Switch from gpt-4-turbo to gpt-3.5-turbo (less accurate)

## Production Deployment

### Backend Deployment (Railway, Render, or Fly.io)

1. Create account on your chosen platform
2. Connect GitHub repository
3. Set environment variables (all from .env.example)
4. Deploy backend service
5. Note the production URL (e.g., `https://your-app.up.railway.app`)

### Frontend Deployment (GitHub Pages)

1. Update `API_BASE_URL` in ChatWidget to your backend URL
2. Update `CORS_ORIGINS` in backend .env to include GitHub Pages URL
3. Build and deploy:
```bash
cd frontend
npm run build
npm run deploy
```

## API Endpoints

The chatbot uses these backend endpoints:

- `POST /api/chat/query` - Submit a question and get RAG response
- `GET /api/chat/health` - Check chat service status

### Example API Call

```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "max_results": 3,
    "include_sources": true
  }'
```

## Cost Estimation

**Per Query Costs** (using OpenRouter):
- Embedding generation: ~$0.0001
- Response generation: ~$0.001-0.005
- **Total per query**: ~$0.002-0.01

**Monthly Estimates** (1000 queries/month):
- $2-10/month for OpenAI API calls
- Qdrant: Free tier (up to 1GB)
- Neon Postgres: Free tier
- Backend hosting: $5-20/month

## Phase 5 Complete!

You now have a fully functional RAG chatbot that:
- Answers questions using textbook content
- Provides source citations
- Supports dark mode
- Works on mobile devices

## Next Steps

- **Phase 6**: Add conversation history tracking
- **Phase 7**: Implement context-aware follow-up questions
- **Phase 8**: Multi-agent coordination for complex queries

## Support

For issues or questions:
- Check backend logs: `python -m src.main` output
- Check frontend console: Browser DevTools
- Review Qdrant collection: `http://localhost:6333/dashboard`
