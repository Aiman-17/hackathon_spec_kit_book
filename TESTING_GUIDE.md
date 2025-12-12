# Testing Guide - AI Robotics Textbook RAG System

This guide explains how to test the complete system locally before deployment.

## Prerequisites

1. **Backend Dependencies**:
```bash
cd backend
pip install -r requirements.txt
pip install pytest pytest-asyncio pytest-cov
```

2. **Environment Variables** (create `backend/.env`):
```env
# OpenAI
OPENAI_API_KEY=your_openai_key

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key

# PostgreSQL Database
NEON_DB_URL=postgresql://user:pass@host/db

# Application
ENVIRONMENT=development
LOG_LEVEL=INFO
```

3. **Data Ingestion** (must be done before testing):
```bash
cd backend
# Ingest all 25 chapters into Qdrant
python scripts/ingest_all_chapters.py
```

---

## Backend Testing

### 1. Run All Integration Tests

```bash
cd backend
pytest tests/integration/ -v -s
```

Expected output:
- `test_rag_pipeline.py`: 10 test queries (≥8 must pass for 80% success rate)
- `test_translation.py`: 4-5 tests (all should pass)

### 2. Run RAG Query Tests (SC-008 Validation)

```bash
cd backend
python tests/test_rag_queries.py
```

This runs the 10 predefined queries from Success Criteria SC-008:
1. "What is ROS 2?" → Must cite "Introduction to Robot Operating System (ROS 2)"
2. "How do URDF and XACRO differ?" → Must cite "URDF & XACRO for Robot Modeling"
3. "Explain the Nav2 navigation stack" → Must cite "Nav2 Navigation Stack"
4. "What sensors are used in humanoid robots?" → Must cite "Robot Hardware"
5. "How does Isaac Sim work?" → Must cite "NVIDIA Isaac Sim"
6. "What is bipedal control?" → Must cite "Motion Planning for Humanoids"
7. "Explain vision-language-action pipelines" → Must cite "Vision-Language-Action Pipelines"
8. "How do ROS nodes communicate?" → Must cite "ROS 2 in Depth"
9. "What is VSLAM?" → Must cite "Isaac ROS Perception Pipelines"
10. "How do you build a humanoid model?" → Must cite "Building a Humanoid Model in Gazebo"

**Success Criteria**: ≥8 of 10 queries must return answer + citation (80% pass rate)

### 3. Test Translation Service

```bash
cd backend
pytest tests/integration/test_translation.py -v -s
```

Validates:
- Markdown structure preservation
- Urdu UTF-8 encoding
- Code block preservation
- Translation speed (<10 seconds)
- Caching functionality

### 4. Manual API Testing

Start the backend server:
```bash
cd backend
python -m uvicorn src.main:app --reload --port 8000
```

Test endpoints:

**Health Check**:
```bash
curl http://localhost:8000/health
```

**RAG Query**:
```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "max_results": 3,
    "include_sources": true
  }'
```

**Translation**:
```bash
curl -X POST http://localhost:8000/api/translate/urdu \
  -H "Content-Type: application/json" \
  -d '{
    "text": "## Introduction\n\nRobotics is the study of robots."
  }'
```

---

## Frontend Testing

### 1. Install Dependencies

```bash
cd frontend
npm install
```

### 2. Run Development Server

```bash
npm start
```

Visit: http://localhost:3000

### 3. Manual Testing Checklist

- [ ] Homepage loads correctly
- [ ] All 4 modules visible in sidebar
- [ ] Navigate to a chapter (e.g., Module 1 → Chapter 1)
- [ ] ChatWidget appears in bottom-right corner
- [ ] Click chat icon to open widget
- [ ] Send test query: "What is ROS 2?"
- [ ] Verify answer appears with citations
- [ ] Check that sources show chapter/section names
- [ ] Test selected-text mode (select text, then ask question)
- [ ] Verify mobile responsiveness (resize window to 375px width)

### 4. Build Test

```bash
cd frontend
npm run build
```

Expected: Zero errors, zero warnings

---

## Integration Testing (Full Stack)

### 1. Start Both Services

**Terminal 1 - Backend**:
```bash
cd backend
python -m uvicorn src.main:app --reload --port 8000
```

**Terminal 2 - Frontend**:
```bash
cd frontend
npm start
```

### 2. End-to-End Test Flow

1. Open http://localhost:3000
2. Navigate to "Module 1" → "Introduction to Physical AI"
3. Open chatbot widget (bottom-right)
4. Ask: "What is Physical AI?"
5. Verify:
   - Answer appears within 2 seconds
   - Answer includes citation
   - Source shows correct chapter
6. Test selected text:
   - Select a paragraph from the chapter
   - Ask: "Explain this in simple terms"
   - Verify answer references selected text
7. Test out-of-scope:
   - Ask: "What is underwater robotics?"
   - Verify: "I don't have information about that" response

### 3. Performance Testing

Check backend logs for response times:
- RAG queries should be < 2 seconds (p95)
- Translation should be < 10 seconds per chapter

---

## Deployment Verification

After deploying to HuggingFace Spaces:

### 1. Check Backend Health

Visit: https://your-space.hf.space/health

Expected response:
```json
{
  "status": "healthy",
  "service": "ai-textbook-backend",
  "version": "1.0.0",
  "features": {
    "personalization": true,
    "translation": true,
    "agent_skills": false
  }
}
```

### 2. Test RAG API

```bash
curl -X POST https://your-space.hf.space/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

### 3. Test Frontend Integration

1. Update frontend/.env with production backend URL:
```env
REACT_APP_BACKEND_URL=https://your-space.hf.space
```

2. Rebuild and deploy frontend to GitHub Pages

3. Visit https://your-username.github.io/repo-name

4. Test chatbot widget with production backend

---

## Success Criteria Checklist

Based on `specs/001-ai-robotics-textbook/spec.md`:

### SC-008: RAG Chatbot (REQUIRED)
- [ ] 8 of 10 test queries return answer + citation (≥80%)
- [ ] Queries complete within 2 seconds (p95)
- [ ] Citations include chapter title and section

### SC-012: Urdu Translation (BONUS)
- [ ] Translation preserves Markdown structure
- [ ] Valid UTF-8 Urdu output
- [ ] Translation completes within 10 seconds
- [ ] Toggle back to English works

### SC-002: Deployment
- [ ] GitHub Pages deployment live
- [ ] HTTPS enabled
- [ ] No mixed content warnings

### Performance (NFR-005)
- [ ] Page load FCP < 1.5 seconds
- [ ] RAG response p95 < 2 seconds
- [ ] Chatbot widget loads < 500ms

---

## Troubleshooting

### "No chunks retrieved" Error

**Problem**: Qdrant has no data

**Solution**:
```bash
cd backend
python scripts/ingest_all_chapters.py
```

### Translation Service Fails

**Problem**: OpenAI API key invalid or expired

**Solution**: Update `OPENAI_API_KEY` in backend/.env

### CORS Errors in Browser

**Problem**: Frontend can't connect to backend

**Solution**: Check backend CORS settings in `src/api/middleware.py`
- Add frontend URL to allowed origins
- For local dev: http://localhost:3000
- For prod: https://your-username.github.io

### Tests Timeout

**Problem**: API calls taking too long

**Possible Causes**:
1. Slow network connection to Qdrant/OpenAI
2. Cold start on serverless backend
3. Large embeddings taking time

**Solution**: Increase pytest timeout or use `--timeout=30` flag

---

## Automated Test Script

Run all tests in sequence:

```bash
#!/bin/bash
# test_all.sh

echo "=== Running Backend Tests ==="
cd backend
pytest tests/integration/ -v --tb=short

echo "=== Running RAG Query Tests ==="
python tests/test_rag_queries.py

echo "=== Building Frontend ==="
cd ../frontend
npm run build

echo "=== All Tests Complete ==="
```

Make executable and run:
```bash
chmod +x test_all.sh
./test_all.sh
```

---

## Reporting Issues

When reporting test failures, include:
1. Test name and file
2. Error message and stack trace
3. Environment (OS, Python version, Node version)
4. `.env` configuration (redact API keys!)
5. Backend logs from uvicorn

Example:
```
Test: test_rag_query_with_citation[Q1]
File: tests/integration/test_rag_pipeline.py
Error: AssertionError: No chunks retrieved for query: What is ROS 2?
Environment: Windows 10, Python 3.11, Node 18
Issue: Qdrant collection is empty - need to run ingestion script
```

---

For more information, see:
- [CHATBOT_LOCAL_SETUP.md](CHATBOT_LOCAL_SETUP.md)
- [HF_DEPLOY_GUIDE.md](HF_DEPLOY_GUIDE.md)
- [specs/001-ai-robotics-textbook/spec.md](specs/001-ai-robotics-textbook/spec.md)
