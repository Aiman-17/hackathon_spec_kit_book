# Deployment Guide

Complete guide for deploying the AI-Native Textbook & RAG System to production.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [GitHub Pages Setup](#github-pages-setup)
3. [Backend Deployment](#backend-deployment)
4. [Environment Configuration](#environment-configuration)
5. [First Deployment](#first-deployment)
6. [Continuous Deployment](#continuous-deployment)
7. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Required Accounts
- [x] GitHub account
- [x] OpenAI API account (for embeddings and RAG)
- [x] Qdrant Cloud account (Free Tier: 1GB, 1M vectors)
- [x] Neon Postgres account (Free Tier: 512MB, 10GB bandwidth/month)
- [ ] Railway/Render/Vercel account (for backend deployment)

### Local Development Tools
- Node.js 18+ and npm
- Python 3.11+
- Git

---

## GitHub Pages Setup

### Step 1: Create GitHub Repository

1. **Create a new repository** on GitHub:
   ```bash
   # Option 1: Via GitHub CLI
   gh repo create yourusername/hackathon_spec_kit_book --public

   # Option 2: Via GitHub web interface
   # Go to https://github.com/new
   # Name: hackathon_spec_kit_book
   # Public repository
   # Do NOT initialize with README (we already have one)
   ```

2. **Update repository URLs** in your local code:

   **File: `frontend/docusaurus.config.js`**
   ```javascript
   // Line 14: Update with your GitHub username
   url: 'https://github.com/Aiman-17',

   // Line 17: Confirm baseUrl matches repo name
   baseUrl: '/hackathon_spec_kit_book/',

   // Line 20: Update organization name
   organizationName: 'Aiman-17',

   // Line 21: Confirm project name
   projectName: 'hackathon_spec_kit_book',
   ```

3. **Push code to GitHub**:
   ```bash
   # Initialize git (if not already done)
   git init

   # Add all files
   git add .

   # Initial commit
   git commit -m "Initial commit: AI-Native Textbook & RAG System"

   # Add remote
   git remote add origin https://github.com/Aiman-17/hackathon_spec_kit_book.git

   # Push to main branch
   git branch -M main
   git push -u origin main
   ```

### Step 2: Enable GitHub Pages

1. Go to your repository on GitHub
2. Click **Settings** tab
3. Scroll to **Pages** section (left sidebar)
4. Under **Source**, select:
   - Source: **GitHub Actions** (recommended)
   - Or select: **Deploy from a branch** → **gh-pages**

5. Click **Save**

### Step 3: Configure GitHub Actions

The workflow is already configured in `.github/workflows/deploy.yml`.

**What it does:**
- Triggers on push to `main` branch
- Installs Node.js dependencies
- Builds the Docusaurus site
- Deploys to GitHub Pages

**Required Permissions** (already configured):
```yaml
permissions:
  contents: read
  pages: write
  id-token: write
```

### Step 4: Verify Deployment

1. After pushing to `main`, go to **Actions** tab
2. Watch the "Deploy to GitHub Pages" workflow
3. Once complete (green checkmark), visit:
   ```
   https://yourusername.github.io/hackathon_spec_kit_book/
   ```

**Expected timeline:** 2-5 minutes for first deployment

---

## Backend Deployment

### Option 1: Railway.app (Recommended)

**Pros:** Easy PostgreSQL + Redis, automatic deployments, free tier available

1. **Sign up** at https://railway.app

2. **Create new project** from GitHub repo:
   ```
   New Project → Deploy from GitHub → hackathon_spec_kit_book
   ```

3. **Add services:**
   - PostgreSQL (Railway provides this)
   - Redis (Railway provides this)

4. **Configure environment variables:**
   ```bash
   # Railway will auto-populate DATABASE_URL
   # Add the following manually:
   OPENAI_API_KEY=sk-your-key-here
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your-qdrant-key
   BETTER_AUTH_SECRET=$(openssl rand -hex 32)
   JWT_SECRET_KEY=$(openssl rand -hex 32)
   CORS_ORIGINS=https://yourusername.github.io
   ENVIRONMENT=production
   ```

5. **Deploy backend:**
   - Railway auto-deploys on git push
   - Or use: `railway up`

### Option 2: Render.com

**Pros:** Simple setup, free tier with PostgreSQL

1. **Sign up** at https://render.com

2. **Create Web Service:**
   - Connect GitHub repository
   - Root directory: `backend`
   - Build command: `pip install -r requirements.txt`
   - Start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`

3. **Add PostgreSQL database** (free tier)

4. **Set environment variables** (same as Railway above)

### Option 3: Vercel (Serverless)

**Note:** Requires adapting FastAPI to Vercel's serverless format

1. Install Vercel CLI:
   ```bash
   npm install -g vercel
   ```

2. Deploy:
   ```bash
   cd backend
   vercel
   ```

3. Configure `vercel.json`:
   ```json
   {
     "builds": [
       {
         "src": "src/main.py",
         "use": "@vercel/python"
       }
     ],
     "routes": [
       {
         "src": "/(.*)",
         "dest": "src/main.py"
       }
     ]
   }
   ```

---

## Environment Configuration

### Frontend (.env - Optional)

Frontend environment variables are configured via `docusaurus.config.js`.

**For API endpoint configuration:**
```env
# frontend/.env (create if needed)
DOCUSAURUS_API_URL=https://your-backend.railway.app
```

### Backend (.env - REQUIRED)

**NEVER commit `.env` to Git!** Use `.env.example` as template.

```bash
# Copy template
cp backend/.env.example backend/.env

# Edit with your values
nano backend/.env
```

**Required values:**
```env
# Neon Postgres
NEON_DB_URL=postgresql://user:password@ep-xxxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# Qdrant Cloud
QDRANT_URL=https://xxxx-xxxx-xxxx.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key

# OpenAI
OPENAI_API_KEY=sk-your_openai_key

# Auth (generate with: openssl rand -hex 32)
BETTER_AUTH_SECRET=your_32_character_secret
JWT_SECRET_KEY=your_jwt_secret

# CORS (allow frontend domain)
CORS_ORIGINS=https://yourusername.github.io,http://localhost:3000

# Environment
ENVIRONMENT=production
DEBUG=false
```

---

## First Deployment

### 1. Frontend Deployment

```bash
# Ensure correct configuration
cd frontend

# Test build locally
npm run build

# If build succeeds, push to GitHub
git add .
git commit -m "feat: initial frontend deployment"
git push origin main

# GitHub Actions will auto-deploy to Pages
```

### 2. Backend Deployment

```bash
# Test backend locally first
cd backend
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -r requirements.txt
uvicorn src.main:app --reload

# Test health endpoint
curl http://localhost:8000/health

# If working, deploy to Railway/Render
git push origin main  # Auto-deploys if connected
```

### 3. Ingest Content

After backend is deployed, ingest textbook content:

```bash
# Generate chapters (optional - or write manually)
python scripts/generate-chapters.py --all

# Validate content quality
python scripts/validate-content.py --all

# Ingest embeddings into Qdrant
python scripts/ingest-embeddings.py --all
```

### 4. Verify Deployment

**Frontend checks:**
- [ ] Visit `https://yourusername.github.io/hackathon_spec_kit_book/`
- [ ] Navigation works
- [ ] Theme loads correctly
- [ ] Content displays properly

**Backend checks:**
- [ ] Visit `https://your-backend.railway.app/health`
- [ ] Should return `{"status":"healthy",...}`
- [ ] API docs: `https://your-backend.railway.app/api/docs` (if DEBUG=true)

---

## Continuous Deployment

### Automated Deployment Workflow

1. **Make changes locally**
2. **Test locally:**
   ```bash
   # Frontend
   cd frontend && npm start

   # Backend
   cd backend && uvicorn src.main:app --reload
   ```

3. **Commit and push:**
   ```bash
   git add .
   git commit -m "feat: add new chapter on SLAM"
   git push origin main
   ```

4. **Automatic deployment:**
   - GitHub Actions builds and deploys frontend
   - Railway/Render auto-deploys backend
   - No manual steps required!

### Branch Strategy

**Main branch:** Production-ready code only
**Feature branches:** For development

```bash
# Create feature branch
git checkout -b feature/add-module-5

# Make changes...
git add .
git commit -m "feat: add Module 5 content"

# Push feature branch
git push origin feature/add-module-5

# Create pull request on GitHub
# After review, merge to main → auto-deploys
```

---

## Troubleshooting

### Frontend Issues

**Problem:** Build fails with "Module not found"
```bash
# Solution: Install dependencies
cd frontend
npm ci  # Clean install
npm run build
```

**Problem:** Site loads but styles missing
```bash
# Solution: Check baseUrl in docusaurus.config.js
# Must match GitHub Pages URL structure
```

**Problem:** 404 on GitHub Pages
```bash
# Solution: Verify GitHub Pages settings
# Source: GitHub Actions
# Branch: gh-pages (if using branch deployment)
```

### Backend Issues

**Problem:** "Missing environment variable" error
```bash
# Solution: Verify .env file exists and has all required variables
cp .env.example .env
# Fill in actual values
```

**Problem:** Database connection fails
```bash
# Solution: Check Neon Postgres connection string
# Ensure ?sslmode=require is present
# Verify IP is whitelisted in Neon dashboard
```

**Problem:** Qdrant upload fails
```bash
# Solution: Verify Qdrant API key and URL
# Check collection exists: scripts/ingest-embeddings.py --reset
```

### GitHub Actions Issues

**Problem:** Workflow fails with permission error
```bash
# Solution: Check repository settings
# Settings → Actions → General → Workflow permissions
# Select: Read and write permissions
```

**Problem:** Deploy job skipped
```bash
# Solution: Ensure branch is 'main' not 'master'
# Workflow triggers on: push to main
```

---

## Production Checklist

Before going live:

**Frontend:**
- [ ] Update all `yourusername` references to actual GitHub username
- [ ] Test build locally: `npm run build`
- [ ] Verify `baseUrl` in `docusaurus.config.js`
- [ ] Enable HTTPS (GitHub Pages provides this automatically)

**Backend:**
- [ ] Set `ENVIRONMENT=production`
- [ ] Set `DEBUG=false`
- [ ] Generate secure secrets (32+ characters)
- [ ] Configure CORS_ORIGINS with production domain
- [ ] Set up monitoring (e.g., Sentry, LogRocket)
- [ ] Configure rate limiting appropriately

**Database:**
- [ ] Neon Postgres connection string configured
- [ ] Database schema initialized
- [ ] Backups enabled (Neon provides automatic backups)

**Vector Database:**
- [ ] Qdrant collection created
- [ ] Embeddings ingested successfully
- [ ] Test search functionality

**Security:**
- [ ] No secrets in Git repository
- [ ] Environment variables set in deployment platform
- [ ] HTTPS enabled everywhere
- [ ] API rate limiting configured

**Monitoring:**
- [ ] GitHub Actions status badge added to README
- [ ] Backend health check endpoint working
- [ ] Error logging configured
- [ ] Analytics configured (optional)

---

## Support

**Documentation:**
- Docusaurus: https://docusaurus.io/docs
- FastAPI: https://fastapi.tiangolo.com
- Qdrant: https://qdrant.tech/documentation
- Railway: https://docs.railway.app

**Issues:**
- Report bugs via GitHub Issues
- Check existing issues first

**Community:**
- Discussions tab on GitHub repository
- Stack Overflow (tag: docusaurus, fastapi)

---

## Next Steps

After successful deployment:

1. **Content Generation:**
   - Generate remaining chapters with `scripts/generate-chapters.py`
   - Validate content quality with `scripts/validate-content.py`

2. **Phase 2 Implementation:**
   - Database schema setup
   - User authentication (Better-Auth)
   - Initial API endpoints

3. **Phase 3 Implementation:**
   - Complete textbook content (25 chapters)
   - Interactive exercises and quizzes

4. **Phase 4 Implementation:**
   - RAG chatbot integration
   - Citation system

5. **Phase 5 Implementation:**
   - Personalization features
   - Urdu translation

6. **Phase 6 (BONUS):**
   - Advanced agent intelligence features

---

**Deployment Status:** 🚀 Ready for GitHub Pages!

**Last Updated:** 2025-12-05
