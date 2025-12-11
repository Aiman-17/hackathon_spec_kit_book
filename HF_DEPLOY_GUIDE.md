# Hugging Face Spaces Deployment Guide

## Quick Deploy: Backend to Hugging Face Spaces

### Step 1: Create Hugging Face Space

1. Go to https://huggingface.co/new-space
2. Fill in:
   - **Space name**: `ai-robotics-rag` (or your choice)
   - **License**: MIT
   - **SDK**: **Docker**
   - **Hardware**: CPU Basic (free tier)
   - **Visibility**: Public

### Step 2: Upload Backend Files

The backend is ready at: `C:\Users\ACS\OneDrive\Desktop\ai-robotics-rag-backend`

**Method A: Direct Upload**
1. In your HF Space, click "Files"
2. Upload these files/folders:
   ```
   Dockerfile
   README.md
   app.py
   requirements-hf.txt
   requirements.txt
   .env.example
   src/ (entire folder)
   scripts/ (entire folder)
   tests/ (entire folder)
   verify_qdrant.py
   verify_tables.py
   ```

**Method B: Git Push (Recommended)**
```bash
cd C:\Users\ACS\OneDrive\Desktop\ai-robotics-rag-backend

# Initialize if needed
git status  # Check if already initialized

# Add HF Space as remote
git remote add hf https://huggingface.co/spaces/mksjai/ai-robotics-rag-backend
git branch -M main

# Push to HF
git push -u hf main
```

### Step 3: Configure Secrets in HF Space

Go to your Space Settings → Variables and secrets

**Required Secrets:**
```bash
OPENAI_API_KEY=your_openai_key_here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
NEON_DB_URL=postgresql://user:password@host/dbname
BETTER_AUTH_SECRET=generate_with_openssl_rand_hex_32
JWT_SECRET_KEY=your_jwt_secret
```

**Optional Configuration:**
```bash
ENVIRONMENT=production
CORS_ORIGINS=https://aiman-17.github.io,https://huggingface.co
LOG_LEVEL=INFO
RATE_LIMIT_PER_MINUTE=100
```

### Step 4: Wait for Build

- HF will automatically build using your Dockerfile
- Build time: 5-10 minutes
- Watch the build logs in the Space

### Step 5: Test Your API

Once deployed, your API will be at:
```
https://YOUR-USERNAME-ai-robotics-rag.hf.space
```

Test it:
```bash
curl https://YOUR-USERNAME-ai-robotics-rag.hf.space/health
```

Test chat:
```bash
curl -X POST https://YOUR-USERNAME-ai-robotics-rag.hf.space/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "max_results": 3}'
```

---

## Step 6: Update Frontend to Use HF Backend

### Option A: Environment Variable (Recommended)

Create `frontend/.env.local`:
```bash
REACT_APP_BACKEND_URL=https://YOUR-USERNAME-ai-robotics-rag.hf.space
```

### Option B: Direct Code Update

Edit `frontend/src/components/ChatWidget/index.tsx` line 29-32:

**Before:**
```typescript
const API_BASE_URL = process.env.REACT_APP_BACKEND_URL ||
  (process.env.NODE_ENV === 'production'
    ? 'https://your-username-ai-robotics-rag.hf.space'
    : 'http://localhost:8000');
```

**After:**
```typescript
const API_BASE_URL = process.env.REACT_APP_BACKEND_URL ||
  'https://YOUR-ACTUAL-USERNAME-ai-robotics-rag.hf.space';
```

Replace `YOUR-ACTUAL-USERNAME` with your HF username!

---

## Step 7: Deploy Frontend to GitHub Pages

```bash
cd C:\Users\ACS\OneDrive\Desktop\hackathon_spec_kit_book

# Add changes
git add frontend/src/components/ChatWidget/index.tsx
# or
git add frontend/.env.local

# Commit
git commit -m "Configure chatbot to use HF backend"

# Push to main (triggers GitHub Pages deployment)
git push origin main
```

Wait 2-5 minutes for GitHub Pages to rebuild.

---

## Verification

### 1. Check HF Space
- Visit: https://huggingface.co/spaces/YOUR-USERNAME/ai-robotics-rag
- Should show "Running" status
- Click "App" tab to see API docs

### 2. Check GitHub Pages
- Visit: https://aiman-17.github.io/hackathon_spec_kit_book/
- Click purple chat button (💬)
- Ask: "What is ROS 2?"
- Should get answer with sources!

---

## Troubleshooting

### HF Space Build Fails

**Check build logs** in HF Space

Common issues:
- Missing dependencies in requirements-hf.txt
- Dockerfile errors
- Port configuration (must use port 7860)

### Chatbot Still Shows "Only Available Locally"

1. **Check frontend URL is updated**
   ```bash
   grep -n "your-username" frontend/src/components/ChatWidget/index.tsx
   ```
   Should NOT show placeholder text

2. **Check GitHub Pages deployed**
   - Go to: https://github.com/Aiman-17/hackathon_spec_kit_book/actions
   - Latest workflow should be green ✅

3. **Clear browser cache**
   - Ctrl+Shift+R (hard refresh)
   - Or use incognito mode

### CORS Errors

Check HF Space logs for CORS issues.

Backend already allows:
- `https://aiman-17.github.io`
- `https://huggingface.co`

If needed, add more origins in HF Space secrets:
```bash
CORS_ORIGINS=https://aiman-17.github.io,https://your-custom-domain.com
```

---

## Cost Estimate

**Free Tier (Recommended):**
- HF Spaces: Free (CPU Basic)
- GitHub Pages: Free
- Qdrant Cloud: Free tier (1GB)
- PostgreSQL (Neon): Free tier

**Paid APIs:**
- OpenAI: ~$0.01 per query (embeddings + GPT-4)
- Estimate: ~$1-5/month for moderate use

---

## Next Steps

After successful deployment:

1. **Test thoroughly**: Ask various questions
2. **Monitor usage**: Check HF Space logs
3. **Update content**: Add more chapters to vector DB
4. **Custom domain** (optional): Configure in GitHub Pages settings

---

## Backend Files Location

Complete backend ready at:
```
C:\Users\ACS\OneDrive\Desktop\ai-robotics-rag-backend\
```

Contains:
- ✅ Dockerfile (HF Spaces compatible)
- ✅ app.py (entry point)
- ✅ requirements-hf.txt (production dependencies)
- ✅ README.md (API documentation)
- ✅ Complete FastAPI application

Just push to HF and configure secrets!
