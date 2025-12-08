# Running the Chatbot Locally

## Quick Start

### Step 1: Start the Backend

```bash
# Navigate to backend directory
cd backend

# Install dependencies (first time only)
pip install -r requirements.txt

# Create .env file with your API keys
cp .env.example .env
# Edit .env and add your keys:
# - OPENAI_API_KEY
# - QDRANT_URL
# - QDRANT_API_KEY
# - NEON_DB_URL
# - etc.

# Run the backend server
python -m uvicorn src.main:app --reload --port 8000
```

The backend should now be running at: http://localhost:8000

### Step 2: Start the Frontend

```bash
# In a NEW terminal, navigate to frontend
cd frontend

# Install dependencies (first time only)
npm install

# Start development server
npm start
```

The site will open at: http://localhost:3000

### Step 3: Test the Chatbot

1. Click the chat icon in the bottom-right corner
2. Ask: "What is ROS 2?"
3. You should get an answer with source citations!

---

## Option 2: Use Production Backend (Skip Local Setup)

If you don't want to run the backend locally, you can configure the frontend to use your deployed Hugging Face backend even in development:

### Update ChatWidget Configuration

Edit `frontend/src/components/ChatWidget/index.tsx`:

```typescript
// Change this line:
const API_BASE_URL = process.env.REACT_APP_BACKEND_URL ||
  (process.env.NODE_ENV === 'production'
    ? 'https://your-username-ai-robotics-rag.hf.space'
    : 'http://localhost:8000');

// To always use HF backend:
const API_BASE_URL = 'https://your-hf-space-url.hf.space';
```

Or create a `.env` file in `frontend/`:

```bash
REACT_APP_BACKEND_URL=https://your-hf-space-url.hf.space
```

---

## Troubleshooting

### Backend Won't Start

**Error: Missing environment variables**
- Make sure you created `.env` from `.env.example`
- Add all required API keys

**Error: Port 8000 already in use**
```bash
# Use a different port
python -m uvicorn src.main:app --reload --port 8001
```

Then update frontend to use port 8001.

### Chatbot Shows Connection Error

1. **Check backend is running**: Visit http://localhost:8000/health
2. **Check CORS settings**: Backend should allow `http://localhost:3000`
3. **Check browser console**: Press F12 and look for errors

### No Answers from Chatbot

**Qdrant not initialized**
- Make sure you ran the ingestion script:
```bash
cd backend
python scripts/ingest_all_chapters.py
```

**OpenAI API key invalid**
- Check your `.env` file has valid `OPENAI_API_KEY`

---

## Quick Test (Backend Only)

Test the backend without frontend:

```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "max_results": 3}'
```

You should get a JSON response with an answer and sources.
