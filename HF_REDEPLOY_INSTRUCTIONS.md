# HF Space Redeployment Instructions

**Issue:** Your HF Space is running OLD code with 800 max_tokens and no 402 error handling.
**Solution:** Trigger a rebuild to pull the latest code from GitHub.

---

## Quick Fix (Option 1: Force Rebuild)

1. **Go to your HF Space:**
   - https://huggingface.co/spaces/mksjai/ai-robotics-rag-backend

2. **Click "Settings" tab** (top of page)

3. **Scroll to "Factory Reboot"** section

4. **Click "Factory Reboot"** button
   - This will pull latest code from GitHub and rebuild

5. **Monitor build logs:**
   - Go to "Logs" tab
   - Watch for successful build
   - Look for: "Application startup complete"

---

## Option 2: Trigger Rebuild via Git

1. **Make a dummy commit to trigger rebuild:**

```bash
# In your local repo
cd C:\Users\ACS\OneDrive\Desktop\hackathon_spec_kit_book

# Create empty commit
git commit --allow-empty -m "trigger: Force HF Space rebuild"

# Push to GitHub
git push origin main
```

2. **HF Space will auto-detect the push and rebuild**

3. **Wait 2-3 minutes for rebuild to complete**

---

## Option 3: Manual Docker Build (If above fails)

If HF Space isn't auto-deploying from GitHub:

1. **Check Space Settings → Repository:**
   - Verify it's connected to: `Aiman-17/hackathon_spec_kit_book`
   - Verify branch is: `main`
   - If not connected, reconnect the GitHub repo

2. **Verify Dockerfile exists:**
   - Check that `Dockerfile` is in repo root
   - It should reference the backend code

---

## Verify Deployment Success

After rebuild completes, test these endpoints:

### 1. Health Check
```bash
curl https://mksjai-ai-robotics-rag-backend.hf.space/health
```

**Expected:** `{"status": "healthy", ...}`

### 2. Credit Status
```bash
curl https://mksjai-ai-robotics-rag-backend.hf.space/api/chat/credit-status
```

**Expected:** JSON with credit status or graceful error

### 3. Test Chat Query
```bash
curl -X POST https://mksjai-ai-robotics-rag-backend.hf.space/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "max_results": 3,
    "include_sources": true,
    "user_level": "student",
    "language": "en"
  }'
```

**Expected with credits:**
```json
{
  "answer": "ROS 2 is...",
  "sources": [...]
}
```

**Expected with 402 error (no credits):**
```json
{
  "answer": "⚠️ The AI service is temporarily unavailable due to usage limits.\n\nThe system retrieved relevant textbook sections...",
  "sources": [...]
}
```

**Key Differences After Redeploy:**
- ✅ Returns 200 status (NOT 500)
- ✅ Uses 400 max_tokens (NOT 800)
- ✅ Free model: `meta-llama/llama-3.2-3b-instruct:free`
- ✅ Graceful 402 error handling with fallback message

---

## Check Build Logs

If deployment fails, check logs for errors:

1. **Go to HF Space → Logs tab**

2. **Look for these indicators:**

**Success:**
```
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:7860
```

**Failure:**
```
ERROR: Could not install packages
ERROR: Failed to build
ModuleNotFoundError: ...
```

3. **Common Issues:**

**Missing Environment Variables:**
```
ERROR: OPENAI_API_KEY not set
```
**Fix:** Add in Settings → Repository Secrets

**Qdrant Connection Failed:**
```
ERROR: Could not connect to Qdrant
```
**Fix:** Verify QDRANT_URL and QDRANT_API_KEY in secrets

**PostgreSQL Connection Failed:**
```
ERROR: Could not connect to PostgreSQL
```
**Fix:** Verify NEON_DB_URL in secrets

---

## Environment Variables (Required)

Make sure these are set in **Settings → Repository Secrets:**

```
OPENAI_API_KEY=<your_openrouter_key>
QDRANT_URL=https://0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=<your_qdrant_key>
NEON_DB_URL=<your_neon_postgres_url>
BETTER_AUTH_SECRET=<32_char_secret>
JWT_SECRET_KEY=<jwt_secret>
ENVIRONMENT=production
CORS_ORIGINS=https://aiman-17.github.io
```

---

## Expected Behavior After Fix

### With Sufficient Credits (>400 tokens):
- Query succeeds
- Returns AI-generated answer
- Includes source citations
- HTTP 200 status

### With Insufficient Credits (<400 tokens):
- Query succeeds (NOT 500 error!)
- Returns fallback message: "⚠️ AI service temporarily unavailable..."
- Shows retrieved textbook chunks
- HTTP 200 status
- User can still see relevant content

### No Credits (0 tokens):
- Same as insufficient credits
- Graceful degradation
- No crashes
- HTTP 200 status

---

## Quick Checklist

- [ ] HF Space connected to GitHub repo `Aiman-17/hackathon_spec_kit_book`
- [ ] Space is watching `main` branch
- [ ] All environment secrets are set correctly
- [ ] Triggered rebuild (factory reboot or empty commit)
- [ ] Build logs show success
- [ ] Health endpoint returns 200
- [ ] Test query returns 200 (even with 402 credit error)
- [ ] Error logs no longer show 500 status

---

## Need Help?

If redeployment fails:

1. **Check Space logs** for specific errors
2. **Verify all secrets** are set correctly
3. **Test Qdrant connection** separately
4. **Verify OpenRouter API key** is valid
5. **Check GitHub repo** has latest commits

**Latest commits on main branch:**
```
293d576 - docs: Add deployment summary and status
c1f78fe - refactor: Replace all GPT-4 references with free model
08935a9 - feat: RAG backend stabilization and deployment readiness
```

These commits have:
- Free model configuration
- 402 error handling
- max_tokens=400 (not 800)
- Graceful degradation

---

**Expected Result:**
Backend handles credit exhaustion gracefully, returns 200 with fallback message, never crashes with 500 error.
