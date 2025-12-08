---
title: Physical AI Robotics RAG API
emoji: 🤖
colorFrom: purple
colorTo: blue
sdk: docker
pinned: false
license: mit
---

# Physical AI & Humanoid Robotics - RAG Backend API

This is the backend API for the [Physical AI & Humanoid Robotics Textbook](https://aiman-17.github.io/hackathon_spec_kit_book/), providing RAG (Retrieval-Augmented Generation) capabilities powered by:

- **FastAPI** - High-performance API framework
- **Qdrant** - Vector database for semantic search
- **OpenAI** - Embeddings and chat completion
- **PostgreSQL** - User data and chat history

## Features

- 🤖 **RAG Chat API** - Answer questions using textbook content
- 📚 **Source Citations** - Every answer includes relevant chapter references
- 🎯 **Context-Aware** - Supports selected text for precise answers
- 🔍 **Semantic Search** - Find relevant content using embeddings
- ⚡ **Fast & Scalable** - Async Python with connection pooling

## API Endpoints

### Health Check
```bash
GET /health
```

### Chat Query
```bash
POST /api/chat/query
Content-Type: application/json

{
  "query": "What is ROS 2?",
  "max_results": 3,
  "include_sources": true
}
```

Response:
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is...",
  "sources": [
    {
      "chapter": "Introduction to ROS 2",
      "section": "Architecture",
      "module": "Module 1",
      "content_snippet": "...",
      "similarity_score": 0.85
    }
  ],
  "timestamp": "2025-12-07T..."
}
```

## Environment Variables

Required secrets (configure in Hugging Face Space settings):

```bash
# OpenAI
OPENAI_API_KEY=your_openai_key

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key

# PostgreSQL Database
NEON_DB_URL=postgresql://user:pass@host/db

# Authentication
BETTER_AUTH_SECRET=your_32_char_secret_key_here
JWT_SECRET_KEY=your_jwt_secret

# Application
ENVIRONMENT=production
CORS_ORIGINS=https://aiman-17.github.io
```

## Deployment

This Space is automatically deployed from the GitHub repository. To deploy:

1. Create a new Hugging Face Space
2. Select "Docker" as the SDK
3. Connect your GitHub repository
4. Configure the secrets in Space settings
5. The Space will auto-deploy on push

## Local Development

```bash
# Install dependencies
pip install -r requirements-hf.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your keys

# Run the server
python app.py
```

## Documentation

- **API Docs**: `/api/docs` (Swagger UI)
- **ReDoc**: `/api/redoc` (Alternative docs)
- **Textbook Site**: https://aiman-17.github.io/hackathon_spec_kit_book/

## License

MIT License - See LICENSE file for details
