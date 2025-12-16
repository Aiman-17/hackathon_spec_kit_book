# Troubleshooting Guide

Common errors and solutions for the Physical AI Robotics RAG API.

## Backend Issues

### 1. OpenRouter Credit Exhaustion

**Symptom:**
- API returns 402 Payment Required
- Chat responses show fallback message: "⚠️ The AI service is temporarily unavailable due to usage limits"

**Solution:**
- Check credit status: `GET /api/chat/credit-status`
- Add credits to OpenRouter account at https://openrouter.ai/credits
- Fallback behavior is intentional - retrieved chunks are still shown to user

**Prevention:**
- Monitor credit usage regularly
- Set up usage alerts in OpenRouter dashboard
- Consider upgrading to paid tier for production

---

### 2. Qdrant Connection Errors

**Symptom:**
- 500 errors on `/api/chat/query`
- Logs show: "Failed to connect to Qdrant"

**Solution:**
```bash
# Verify Qdrant credentials
echo $QDRANT_URL
echo $QDRANT_API_KEY

# Test connection
curl -H "api-key: $QDRANT_API_KEY" $QDRANT_URL/collections
```

**Common causes:**
- Invalid `QDRANT_URL` or `QDRANT_API_KEY` in environment
- Qdrant cluster suspended (free tier inactivity)
- Network firewall blocking HTTPS to Qdrant

---

### 3. PostgreSQL Connection Errors

**Symptom:**
- Backend fails to start
- Logs show: "Could not connect to PostgreSQL"

**Solution:**
```bash
# Verify database URL format
echo $NEON_DB_URL
# Should be: postgresql://user:password@host:port/database

# Test connection
psql $NEON_DB_URL -c "SELECT 1"
```

**Common causes:**
- Missing `NEON_DB_URL` environment variable
- Database suspended (Neon free tier inactivity)
- Incorrect password or host

---

### 4. CORS Errors

**Symptom:**
- Frontend shows: "Access to fetch blocked by CORS policy"
- Browser console shows CORS error

**Solution:**

1. **Local Development:**
   - Ensure backend is running on `http://localhost:8000`
   - Frontend should detect localhost and use correct URL

2. **Production (GitHub Pages):**
   - Add your domain to `CORS_ORIGINS` in backend/.env:
     ```bash
     CORS_ORIGINS=https://yourusername.github.io,https://aiman-17.github.io
     ```
   - Restart backend

3. **Hugging Face Spaces:**
   - CORS is pre-configured for `*.hf.space` and `*.huggingface.co`
   - If issues persist, check `backend/src/config.py` line 61

---

## Frontend Issues

### 5. Blank Chatbot UI

**Symptom:**
- Chatbot widget doesn't appear
- No errors in console

**Solution:**
```bash
# Rebuild frontend
cd frontend
npm run build

# Check for build errors
npm run build 2>&1 | grep ERROR
```

**Common causes:**
- Build failed silently
- Missing `static/` directory
- JavaScript bundle not loaded

---

### 6. API Connection Failures

**Symptom:**
- Chat queries fail with network error
- Console shows: "Failed to fetch"

**Diagnosis:**
```javascript
// Open browser console and check backend URL detection
console.log(window.location.hostname);
```

**Solution:**

1. **Local Development:**
   - Ensure backend is running: `curl http://localhost:8000/health`
   - Check frontend detects localhost correctly

2. **Production:**
   - Verify backend URL in `frontend/src/components/ChatbotWidget/index.js:38-46`
   - Update `getBackendUrl()` if deploying to custom domain

---

### 7. Personalization Settings Not Persisting

**Symptom:**
- User level or language resets on page reload

**Solution:**
```javascript
// Open browser console and check localStorage
console.log(localStorage.getItem('userLevel'));
console.log(localStorage.getItem('language'));
```

**Common causes:**
- Browser in incognito/private mode (localStorage disabled)
- Browser settings blocking localStorage
- Third-party cookie blockers

**Workaround:**
- Use normal browser mode
- Whitelist your domain in browser settings

---

## Deployment Issues

### 8. GitHub Pages Deployment Fails

**Symptom:**
```bash
npm run deploy
ERROR: Please set the GIT_USER environment variable
```

**Solution:**
```bash
# Set GIT_USER before deploying
GIT_USER=your-github-username npm run deploy

# Or export it
export GIT_USER=your-github-username
npm run deploy
```

---

### 9. Hugging Face Space Build Fails

**Symptom:**
- Space shows "Building..." indefinitely
- Build logs show errors

**Common solutions:**

1. **Check Dockerfile:**
   ```bash
   # Verify Dockerfile exists and is valid
   docker build -t test-build .
   ```

2. **Verify requirements:**
   ```bash
   # Check requirements-hf.txt has all dependencies
   cat requirements-hf.txt
   ```

3. **Check secrets:**
   - Go to Space Settings → Repository secrets
   - Verify all required secrets are set:
     - `OPENAI_API_KEY` (OpenRouter key)
     - `QDRANT_URL`
     - `QDRANT_API_KEY`
     - `NEON_DB_URL`
     - `BETTER_AUTH_SECRET`
     - `JWT_SECRET_KEY`

---

### 10. Production API Returns 500 Errors

**Symptom:**
- Chat works locally but fails in production
- Backend logs show unhandled exceptions

**Diagnosis:**
```bash
# Check backend logs
# For HF Spaces: View logs in Space UI
# For custom deployment: check server logs

# Test credit status endpoint
curl https://your-backend-url/api/chat/credit-status
```

**Common causes:**
1. **Missing environment variables** - Check all secrets are set
2. **OpenRouter credits exhausted** - Add credits or wait for free tier reset
3. **Database connection timeout** - Verify `NEON_DB_URL` and database is active
4. **Qdrant cluster suspended** - Reactivate Qdrant cluster

**Solution:**
- Review backend logs for specific error
- Verify all environment variables are set correctly
- Test each service individually (Qdrant, PostgreSQL, OpenRouter)

---

## API Testing

### Quick Health Checks

```bash
# Backend health
curl https://your-backend-url/health

# Credit status
curl https://your-backend-url/api/chat/credit-status

# Test query (replace with your backend URL)
curl -X POST https://your-backend-url/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "max_results": 3,
    "include_sources": true,
    "user_level": "student",
    "language": "en"
  }'
```

---

## Getting Help

If you encounter an issue not listed here:

1. **Check logs:**
   - Backend: Server console or HF Space logs
   - Frontend: Browser console (F12 → Console tab)

2. **Verify configuration:**
   - Environment variables set correctly
   - All services (Qdrant, PostgreSQL, OpenRouter) are active

3. **Test components individually:**
   - Test backend health endpoint
   - Test frontend build locally
   - Test each API endpoint with curl

4. **Report issues:**
   - Include error messages from logs
   - Specify environment (local/production)
   - List steps to reproduce
