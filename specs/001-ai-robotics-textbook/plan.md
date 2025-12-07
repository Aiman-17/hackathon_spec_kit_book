# Implementation Plan: AI-Native Textbook & RAG System for Physical AI & Humanoid Robotics

**Branch**: `001-ai-robotics-textbook` | **Date**: 2025-12-04 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ai-robotics-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a complete Docusaurus-based textbook for Physical AI & Humanoid Robotics (4 modules, 25 chapters) with integrated RAG chatbot powered by FastAPI, Qdrant Cloud, and OpenAI embeddings. Include Better-Auth authentication, personalization based on user skill level, Urdu translation, and optional Claude Subagents for bonus scoring. Deploy frontend to GitHub Pages and backend to free-tier hosting (Render/Railway/HuggingFace Spaces).

**Technical Approach**:
- Content generation using AI agents following chapter template structure
- Semantic chunking by H2/H3 headers (500-1000 token overlap) for optimal RAG retrieval
- Text-embedding-3-small (1536 dims) for cost-efficient vector storage
- Transient personalization caching (session/Redis) to avoid database bloat
- Markdown-preserving translation pipeline

## Technical Context

**Language/Version**:
- Frontend: JavaScript/TypeScript with Node.js 18+ (Docusaurus 3.x, React 18)
- Backend: Python 3.11+ (FastAPI 0.104+)

**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 18, Better-Auth (TypeScript client)
- Backend: FastAPI, OpenAI Python SDK 1.x, Qdrant Client 1.7+, psycopg2/asyncpg (Neon Postgres), Pydantic 2.x
- AI/ML: OpenAI GPT-4 (RAG responses), text-embedding-3-small (embeddings), GPT-4o-mini (translation)

**Storage**:
- Vector DB: Qdrant Cloud Free Tier (1GB, 1M vectors max)
- Relational DB: Neon Postgres Serverless Free Tier (512MB, 10GB bandwidth/month)
- Cache: Redis or in-memory dict for transient personalized content
- Static Assets: GitHub Pages (1GB repo, 100GB bandwidth/month)

**Testing**:
- Frontend: Jest + React Testing Library (unit), Playwright (E2E)
- Backend: pytest + pytest-asyncio (unit/integration), httpx (API client tests)
- RAG Pipeline: Custom test suite with 10 predefined queries (≥70% precision threshold)
- Content Quality: markdownlint + custom validation script (15 criteria checklist)

**Target Platform**:
- Frontend Deployment: GitHub Pages (static site)
- Backend Deployment: Free tier cloud (Render/Railway/HuggingFace Spaces/Vercel)
- Supported Browsers: Chrome 90+, Firefox 88+, Safari 14+, Edge 90+
- Mobile: Responsive design for 320px+ viewports

**Project Type**: Web application (frontend + backend microservices)

**Performance Goals**:
- RAG query response: p95 < 2 seconds
- Page load (FCP): < 1.5 seconds
- Docusaurus build time: ≤ 5 minutes (25 chapters)
- Embedding pipeline: Process all 25 chapters within 10 minutes
- Personalization generation: ≤ 5 seconds per chapter
- Translation request: ≤ 10 seconds per chapter

**Constraints**:
- Free tier limits: Qdrant (1GB), Neon (512MB, 10GB bandwidth), GitHub Pages (1GB repo, 100GB bandwidth)
- Backend cold start: ≤10 seconds acceptable on free tier
- No paid services required for MVP
- Markdown structure must be preserved in translations
- Chunk size: 500-1500 tokens (≥80% of chunks in range)
- Mobile touch targets: ≥44x44px
- Zero build errors/warnings required

**Scale/Scope**:
- Content: 4 modules, 25 chapters, ~50,000 words estimated
- Users: Designed for 100-500 concurrent users (free tier constraints)
- Vectors: ~100-200 chapter chunks in Qdrant
- Database: ~1000-5000 users projected (hackathon scope)
- Repository size: <500MB (including assets)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ **Spec-Driven Authoring**
- All chapters follow defined template (Summary → Objectives → Tutorial → Code → Glossary → Review)
- Content generation via Spec-Kit Plus workflow (spec → plan → tasks → implement)
- No free-form writing; every deliverable follows specification

**Compliance**: PASS - Specification defines exact chapter structure and generation workflow

### ✅ **Technical Accuracy**
- Engineering constraints explicitly stated (free tier limits, cold start times, chunk sizes)
- Physical AI/robotics content aligned with official course outline
- Best practices for ROS 2, NVIDIA Isaac Sim, Gazebo documented

**Compliance**: PASS - Technical constraints quantified in NFRs; content aligned with course spec

### ✅ **Pedagogical Clarity**
- Learning objectives use Bloom's taxonomy verbs (explain, implement, analyze)
- Progressive complexity within chapters (simple → advanced)
- Each chapter includes Prerequisites, Key Takeaways, Glossary (≥5 terms), Review Questions (≥3)

**Compliance**: PASS - SC-004 defines 15-point content quality checklist enforcing pedagogical standards

### ✅ **Chunkable Knowledge**
- Semantic chunking by H2/H3 headers (500-1000 token overlap)
- Self-contained chunks (≥80% understandable independently)
- Explicit terminology (no vague pronouns)
- Each section includes necessary context

**Compliance**: PASS - Clarifications specify semantic chunking strategy; SC-005 validates chunk quality

### ✅ **Agent-Augmented Writing**
- Claude Subagents: Draft Writer, Reviewer, Code Example Generator, ROS Troubleshooter
- Agent Skills: rewrite_content, summarize_for_embedding, personalize_for_user, translate_to_urdu
- Bonus scoring for agent implementation (SC-007, SC-013)

**Compliance**: PASS - Architecture section defines agent layer; FR-019/FR-020 specify agent requirements

### ✅ **Clean Markdown for Docusaurus**
- Output to /docs with correct frontmatter
- H2/H3/H4 heading hierarchy
- Code blocks with language tags (```python, ```bash)
- No inline HTML (unless Docusaurus-required)

**Compliance**: PASS - FR-002, FR-003 enforce Docusaurus compatibility; SC-004 validates structure

### ✅ **Multi-Pass Reviews**
- Content quality checklist (15 criteria): structure, technical quality, pedagogical quality
- Automated: markdownlint + custom validation script
- Manual: Spot-check 5 random chapters

**Compliance**: PASS - SC-004 defines review workflow; threshold ≥23 of 25 chapters pass

### ⚠️ **Minimum 20 Chapters, Maximum 50**
- Planned: Exactly 25 chapters across 4 modules
- Module 1: 6 chapters
- Module 2: 6 chapters
- Module 3: 7 chapters
- Module 4: 6 chapters

**Compliance**: PASS - Within constitution range (20-50); aligned with course outline

### ✅ **GitHub Pages Deployment**
- GitHub Actions workflow for automated deployment
- Live public URL required (SC-002)
- Trigger on every commit to main branch

**Compliance**: PASS - Deployment section specifies GitHub Actions workflow; SC-002 validates deployment

### ✅ **RAG Integration**
- Qdrant vector storage with text-embedding-3-small
- Clear chunk boundaries (H2/H3 semantic sections)
- Citations include chapter title + section heading

**Compliance**: PASS - Architecture specifies Qdrant integration; SC-008 validates RAG functionality with citations

---

**Constitution Gate Result**: ✅ **PASS** - All principles and constraints satisfied

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-robotics-textbook/
├── spec.md              # Feature specification (complete with clarifications)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (technology research)
├── data-model.md        # Phase 1 output (entities and schemas)
├── quickstart.md        # Phase 1 output (setup instructions)
├── contracts/           # Phase 1 output (API contracts)
│   ├── openapi.yaml     # FastAPI backend API specification
│   ├── rag.yaml         # RAG endpoints (query, ingest, citations)
│   ├── auth.yaml        # Better-Auth endpoints (signup, signin, profile)
│   ├── personalization.yaml  # Personalization endpoints
│   └── translation.yaml      # Translation endpoints
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Option 2: Web application (frontend + backend)

backend/
├── src/
│   ├── main.py                    # FastAPI application entry point
│   ├── config.py                  # Environment variables + settings
│   ├── models/
│   │   ├── user.py                # User, UserProfile models
│   │   ├── chat.py                # ChatMessage model
│   │   └── chapter.py             # ChapterChunk model
│   ├── services/
│   │   ├── document_ingestion.py  # Parse Markdown, chunk by H2/H3
│   │   ├── embedding_pipeline.py  # OpenAI text-embedding-3-small integration
│   │   ├── qdrant_manager.py      # Vector DB operations
│   │   ├── postgres_manager.py    # Neon Postgres connection pool
│   │   ├── rag_service.py         # Query processing, citation extraction
│   │   ├── personalization.py     # Content adaptation logic
│   │   └── translation.py         # Urdu translation with Markdown preservation
│   ├── api/
│   │   ├── routers/
│   │   │   ├── chat.py            # RAG query endpoints
│   │   │   ├── auth.py            # Better-Auth integration
│   │   │   ├── personalize.py     # Personalization endpoints
│   │   │   ├── translate.py       # Translation endpoints
│   │   │   └── ingest.py          # Admin endpoint for embedding ingestion
│   │   └── middleware.py          # CORS, rate limiting, auth
│   ├── agents/                    # Claude Subagents (bonus)
│   │   ├── draft_writer.py
│   │   ├── reviewer.py
│   │   ├── code_generator.py
│   │   └── ros_troubleshooter.py
│   └── skills/                    # Agent Skills (bonus)
│       ├── rewrite_content.py
│       ├── summarize_for_embedding.py
│       ├── personalize_for_user.py
│       └── translate_to_urdu.py
├── tests/
│   ├── unit/
│   │   ├── test_chunking.py       # Semantic chunking validation
│   │   ├── test_embeddings.py     # Embedding pipeline
│   │   └── test_personalization.py
│   ├── integration/
│   │   ├── test_qdrant.py         # Vector DB integration
│   │   ├── test_postgres.py       # Neon Postgres
│   │   └── test_rag_pipeline.py   # End-to-end RAG flow
│   └── contract/
│       └── test_api.py            # OpenAPI contract validation
├── requirements.txt               # Python dependencies
├── .env.example                   # Environment variable template
└── README.md

frontend/
├── docs/                          # Docusaurus content
│   ├── intro.md
│   ├── module-1/                  # Foundations of Physical AI & Robotics
│   │   ├── 01-introduction-to-physical-ai.md
│   │   ├── 02-humanoid-robotics-overview.md
│   │   ├── 03-robot-hardware.md
│   │   ├── 04-mathematics-for-robotics.md
│   │   ├── 05-introduction-to-ros2.md
│   │   └── 06-linear-algebra-calculus.md
│   ├── module-2/                  # Simulation Environments & Robotics Software
│   │   ├── 07-ros2-in-depth.md
│   │   ├── 08-building-robot-applications.md
│   │   ├── 09-urdf-xacro.md
│   │   ├── 10-gazebo-comparison.md
│   │   ├── 11-building-humanoid-gazebo.md
│   │   └── 12-unity-visualization.md
│   ├── module-3/                  # Advanced Perception, Navigation & Control
│   │   ├── 13-computer-vision-robotics.md
│   │   ├── 14-isaac-sim-fundamentals.md
│   │   ├── 15-isaac-ros-pipelines.md
│   │   ├── 16-nav2-navigation.md
│   │   ├── 17-mapping-localization.md
│   │   ├── 18-motion-planning-humanoids.md
│   │   └── 19-vision-language-action.md
│   └── module-4/                  # Humanoid AI Systems & Capstone
│       ├── 20-integrating-perception-action.md
│       ├── 21-ai-agents-robotics.md
│       ├── 22-end-to-end-humanoid.md
│       ├── 23-multi-agent-coordination.md
│       ├── 24-project-autonomous-simulation.md
│       └── 25-final-capstone.md
├── src/
│   ├── components/
│   │   ├── ChatbotWidget.tsx      # Embedded RAG chatbot
│   │   ├── PersonalizeButton.tsx  # "Personalize this Chapter"
│   │   ├── TranslateButton.tsx    # "Translate to Urdu"
│   │   ├── SelectedTextQuery.tsx  # "Ask based on selected text"
│   │   └── CitationPreview.tsx    # Expandable citation inline preview
│   ├── pages/
│   │   ├── index.tsx              # Homepage with hero section
│   │   └── auth/
│   │       ├── signup.tsx         # Better-Auth signup form
│   │       └── signin.tsx         # Better-Auth signin form
│   ├── services/
│   │   ├── api.ts                 # Backend API client (axios/fetch)
│   │   ├── auth.ts                # Better-Auth TypeScript client
│   │   └── chatbot.ts             # Chatbot WebSocket/HTTP client
│   ├── styles/
│   │   └── futuristic-theme.css   # Navy/Cyan/Purple color palette, neon glows
│   └── theme/
│       └── custom.css             # Docusaurus theme overrides
├── static/
│   ├── img/                       # Logo, icons, diagrams
│   └── fonts/                     # Orbitron, Fira Code fonts
├── docusaurus.config.js           # Docusaurus configuration
├── sidebars.js                    # Sidebar navigation structure
├── package.json
└── tsconfig.json

.github/
└── workflows/
    └── deploy.yml                 # GitHub Actions: build + deploy to GitHub Pages

docs-source/                       # Input: Course outline Word doc
└── course-outline.docx

scripts/
├── generate-chapters.py           # AI-powered chapter generation script
├── ingest-embeddings.py           # Qdrant ingestion script
└── validate-content.py            # Content quality checker (15 criteria)
```

**Structure Decision**: Web application with separate frontend (Docusaurus + React) and backend (FastAPI microservices). Frontend deployed to GitHub Pages (static build), backend deployed to free-tier cloud hosting. Docs organized by module folders matching course outline structure. Backend services modularized by domain (RAG, auth, personalization, translation) for clarity and independent testing.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected** - All constitution principles and constraints are satisfied by the current design.

---

# Phase 0: Research & Technology Validation

## Overview

Phase 0 resolves all "NEEDS CLARIFICATION" items from Technical Context and validates technology choices through research. Since all Technical Context fields are already specified, this phase focuses on validating integration patterns and best practices for the chosen tech stack.

## Research Tasks

### 1. Docusaurus 3.x + Better-Auth Integration Pattern

**Decision**: Use Better-Auth React hooks with Docusaurus custom React components

**Rationale**:
- Better-Auth provides TypeScript-first React hooks (`useAuth()`, `useSession()`)
- Docusaurus supports custom React components in MDX files via `@theme` swizzling
- Auth state can be managed via React Context Provider wrapped around Docusaurus app

**Integration Approach**:
1. Install Better-Auth React client: `npm install better-auth`
2. Create `src/components/AuthProvider.tsx` wrapping Docusaurus theme
3. Add auth context to `docusaurus.config.js` via `clientModules`
4. Signup/signin pages use Better-Auth hooks + FastAPI backend endpoints

**Alternatives Considered**:
- NextAuth.js: Designed for Next.js, not Docusaurus-native
- Firebase Auth: Requires Google Cloud, violates "no paid services" constraint
- Custom JWT: Reinventing wheel, Better-Auth provides battle-tested security

**Reference**: [Better-Auth Docusaurus integration guide](https://better-auth.com/docs/integrations/docusaurus)

### 2. FastAPI + Qdrant Cloud Semantic Search Best Practices

**Decision**: Use `qdrant-client` Python SDK with async operations + vector filtering

**Rationale**:
- Qdrant Python client provides async/await support for FastAPI compatibility
- Payload filtering enables metadata-based search (e.g., filter by module/chapter)
- HNSW index provides <100ms search latency for 100-200 vectors at Free Tier scale

**Implementation Pattern**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Create collection with text-embedding-3-small dimensions (1536)
client.create_collection(
    collection_name="book_chapters",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
)

# Query with metadata filter
results = client.search(
    collection_name="book_chapters",
    query_vector=embedding,  # from OpenAI text-embedding-3-small
    query_filter=Filter(
        must=[
            FieldCondition(key="module", match={"value": "module-1"})
        ]
    ),
    limit=5,
    with_payload=True  # Return chunk text + metadata for citations
)
```

**Best Practices**:
- Batch embed chunks (max 2048 chunks per request to OpenAI)
- Store metadata in payload: `{"chapter_title": str, "section_heading": str, "module": str, "url": str}`
- Use `scroll()` API for ingestion (handles pagination automatically)
- Enable quantization for free tier (reduces storage by 4x with <5% quality loss)

**Alternatives Considered**:
- Pinecone: Free tier only 1 index, slower cold start
- Weaviate: Requires self-hosting, violates "no infrastructure" constraint
- pgvector (Postgres extension): Neon Free Tier doesn't support extensions

**Reference**: [Qdrant FastAPI tutorial](https://qdrant.tech/documentation/tutorials/rag-using-qdrant-and-openai/)

### 3. Semantic Chunking Strategy (H2/H3 Headers with Overlap)

**Decision**: Use `markdown-it` Python library + custom splitter for semantic chunking

**Rationale**:
- Parse Markdown AST to identify H2/H3 boundaries
- Preserve complete sections (no mid-sentence splits)
- Add 500-1000 token overlap by including previous section's summary

**Implementation**:
```python
import markdown_it
from tiktoken import get_encoding

tokenizer = get_encoding("cl100k_base")  # GPT-4 tokenizer

def chunk_by_headers(markdown_text: str, overlap_tokens: int = 750) -> list[dict]:
    """
    Split Markdown into semantic chunks at H2/H3 boundaries.
    Each chunk includes overlap from previous section.
    """
    md = markdown_it.MarkdownIt()
    tokens = md.parse(markdown_text)

    chunks = []
    current_chunk = {"text": "", "heading": "", "level": 0}
    previous_text = ""

    for token in tokens:
        if token.type == "heading_open" and token.tag in ["h2", "h3"]:
            # Save previous chunk with overlap
            if current_chunk["text"]:
                # Add overlap from previous section
                overlap = get_last_n_tokens(previous_text, overlap_tokens)
                chunk_with_overlap = overlap + "\n\n" + current_chunk["text"]
                chunks.append({
                    "text": chunk_with_overlap,
                    "heading": current_chunk["heading"],
                    "level": current_chunk["level"],
                    "token_count": len(tokenizer.encode(chunk_with_overlap))
                })
                previous_text = current_chunk["text"]

            # Start new chunk
            current_chunk = {
                "text": "",
                "heading": extract_heading_text(tokens),
                "level": int(token.tag[1])  # Extract 2 or 3 from h2/h3
            }
        else:
            current_chunk["text"] += token.content

    return chunks
```

**Chunk Quality Validation**:
- Target: 500-1500 tokens per chunk
- Metric: ≥80% of chunks in target range (SC-005 Metric 2)
- Overlap prevents context loss at boundaries

**Alternatives Considered**:
- Fixed-size chunking (512 tokens): Breaks semantic boundaries, poor citation quality
- Paragraph-based: Too granular, loses H2/H3 structure for RAG
- LangChain RecursiveCharacterTextSplitter: Doesn't respect Markdown structure

**Reference**: [RAG chunking strategies comparison](https://www.pinecone.io/learn/chunking-strategies/)

### 4. Transient Personalization Caching (Session/Redis)

**Decision**: Use Redis with 1-hour TTL for personalized content cache

**Rationale**:
- Personalized chapters are large (5-10KB each)
- Regenerating on every page load wastes OpenAI API credits
- Redis Free Tier (Upstash/Railway): 10MB storage, 10K requests/day sufficient for hackathon
- 1-hour TTL balances freshness vs regeneration cost

**Cache Key Structure**:
```python
# Redis key format
cache_key = f"personalized:{user_id}:{chapter_id}:{profile_hash}"

# profile_hash = sha256(skill_level + technical_background + learning_goals)
# Ensures cache invalidation when profile changes
```

**Implementation**:
```python
import redis
import hashlib

r = redis.Redis(host=REDIS_URL, decode_responses=True)

def get_personalized_chapter(user_id: int, chapter_id: str, profile: UserProfile) -> str:
    profile_hash = hashlib.sha256(
        f"{profile.skill_level}{profile.technical_background}{profile.learning_goals}".encode()
    ).hexdigest()[:8]

    cache_key = f"personalized:{user_id}:{chapter_id}:{profile_hash}"

    # Check cache
    cached = r.get(cache_key)
    if cached:
        return cached

    # Generate personalized content
    personalized_content = personalize_content(chapter_id, profile)

    # Cache for 1 hour
    r.setex(cache_key, 3600, personalized_content)

    return personalized_content
```

**Fallback Strategy**:
- If Redis unavailable: In-memory dict cache (LRU with 100 entry limit)
- If cache miss + OpenAI rate limit: Return default content with notice

**Alternatives Considered**:
- PostgreSQL table: Bloats DB, requires cleanup jobs, slower than Redis
- No caching: Expensive ($0.10/chapter * 25 chapters * 100 users = $250), slow UX
- Browser localStorage: Loses cross-device state, security risk

**Reference**: [Redis caching patterns](https://redis.io/docs/manual/patterns/)

### 5. Urdu Translation with Markdown Preservation

**Decision**: Use OpenAI GPT-4o-mini with custom prompt enforcing Markdown structure

**Rationale**:
- GPT-4o-mini supports Urdu with high quality ($0.15/1M input tokens, 60% cheaper than GPT-4)
- Custom system prompt instructs model to preserve Markdown syntax
- Post-processing validation ensures structure integrity

**Translation Prompt Template**:
```python
TRANSLATION_PROMPT = """
Translate the following Markdown content to Urdu.

CRITICAL RULES:
1. Preserve ALL Markdown formatting:
   - Keep headers: ## stays ##, ### stays ###
   - Keep code blocks: ```python ... ``` unchanged (do NOT translate code)
   - Keep bullet lists: - and 1. formatting
   - Keep bold **text** and italic *text* markers
   - Keep links: [text](url) structure preserved, translate text only
2. Translate only natural language text (headings, paragraphs, list items)
3. Do NOT translate:
   - Code snippets
   - URLs
   - Technical identifiers (e.g., ROS 2, Isaac Sim, URDF)
   - File paths
4. Keep the exact same number of lines and sections

Original Markdown:
{markdown_content}

Translated Urdu Markdown:
"""
```

**Validation**:
```python
import re

def validate_markdown_structure(original: str, translated: str) -> bool:
    """Ensure Markdown structure preserved after translation."""

    # Count structural elements
    checks = [
        (r'^#{2,4}\s', "headers"),  # H2/H3/H4 count must match
        (r'```\w+', "code_blocks"),  # Code block count must match
        (r'^\s*[-*]\s', "bullet_lists"),  # Bullet list count must match
        (r'\[.*?\]\(.*?\)', "links"),  # Link count must match
    ]

    for pattern, element in checks:
        orig_count = len(re.findall(pattern, original, re.MULTILINE))
        trans_count = len(re.findall(pattern, translated, re.MULTILINE))
        if orig_count != trans_count:
            raise ValueError(f"Structure mismatch: {element} count differs")

    return True
```

**Cost Estimation**:
- Average chapter: ~5000 words = ~6500 tokens
- GPT-4o-mini: $0.15/1M input + $0.60/1M output
- Cost per chapter: ~$0.01 input + $0.04 output = $0.05
- Total for 25 chapters: $1.25

**Alternatives Considered**:
- Google Translate API: Breaks Markdown structure, poor technical term quality
- DeepL: Excellent quality but $5/chapter (100x more expensive)
- Manual translation: Not feasible for 25 chapters in hackathon timeline

**Reference**: [OpenAI GPT-4o-mini Urdu capabilities](https://platform.openai.com/docs/models/gpt-4o-mini)

### 6. Better-Auth + Neon Postgres User Profile Schema

**Decision**: Extend Better-Auth default user table with separate `user_profiles` table

**Rationale**:
- Better-Auth manages `users` table (id, email, password_hash, created_at)
- Separate `user_profiles` table for domain-specific fields (skill_level, technical_background, learning_goals)
- 1:1 relationship via foreign key

**Database Schema**:
```sql
-- Better-Auth managed table (do not modify)
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    email_verified BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Custom profile table for personalization
CREATE TABLE user_profiles (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    skill_level VARCHAR(20) CHECK (skill_level IN ('beginner', 'intermediate', 'advanced')),
    technical_background TEXT,  -- Free-text: prior experience with robotics/AI/programming
    learning_goals TEXT,  -- Free-text: career development, academic research, hobby projects
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(user_id)
);

-- Chat history for RAG conversations
CREATE TABLE chat_history (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    citations JSONB,  -- Array of {chapter_title, section, url}
    context_type VARCHAR(20) CHECK (context_type IN ('general', 'selected_text')),
    selected_text TEXT,  -- Null for general queries
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    INDEX idx_user_timestamp (user_id, timestamp DESC)
);
```

**Signup Flow**:
1. User submits email + password → Better-Auth creates `users` record
2. Redirect to `/onboarding` → collect skill_level, technical_background, learning_goals
3. Insert `user_profiles` record with `user_id` foreign key
4. Redirect to `/docs/intro`

**Neon Postgres Free Tier Constraints**:
- 512MB storage: ~5000 users with profiles + 50K chat messages
- 10GB bandwidth/month: ~100K API requests
- Sufficient for hackathon scope (projected 100-500 users)

**Alternatives Considered**:
- Single `users` table with all fields: Violates separation of concerns, couples auth to domain logic
- NoSQL (MongoDB): Neon Postgres Free Tier is sufficient, SQL better for relational queries
- Supabase: Similar to Neon but requires additional setup, no benefit

**Reference**: [Better-Auth database schema](https://better-auth.com/docs/concepts/database-schema)

---

## Research Validation Checklist

✅ **Docusaurus 3.x + Better-Auth**: Integration pattern validated
✅ **FastAPI + Qdrant**: Async client pattern with payload filtering
✅ **Semantic Chunking**: H2/H3 parser with overlap strategy
✅ **Personalization Caching**: Redis with TTL + profile hash invalidation
✅ **Urdu Translation**: GPT-4o-mini with Markdown preservation prompt
✅ **User Profile Schema**: 1:1 extension of Better-Auth users table

**Phase 0 Complete** - All technology choices validated with implementation patterns documented.

---

**(Continuing to Phase 1 in next section...)**
